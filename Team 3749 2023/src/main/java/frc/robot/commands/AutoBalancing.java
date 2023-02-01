// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Noah Simon
 * 
 *         Allows the robot to automatically "engage" on the charging station
 */
public class AutoBalancing extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    SwerveSubsystem drivetrain;

    double angle = drivetrain.getVerticalTilt();
    PIDController controller = new PIDController(0.5, 0, 0);

    // Initializes the BaseCommand
    public AutoBalancing(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Run on command init
    @Override
    public void initialize() {
        drivetrain.stopModules();
    }

    // Run every 20 ms
    @Override
    public void execute() {
         // make 1 a constant. But it's supposed to be how much of not straight forward we'll allow, Should prob be lower than 1 degree if possible
        if (Math.abs(drivetrain.getHeading())>Constants.AutoBalancing.max_yaw_offset){
            drivetrain.turnToDegrees(0);

        }

        else{
            angle = drivetrain.getVerticalTilt();
            // 2 is for the midpoint, the 15 degree to radian is to map 15 degrees to 1, 4*0.3048 is the length of the station to meters
            double angleConst = 2*(1 - Math.cos(15/360*2*Math.PI))/(4*0.3048);

            double dist = (1-Math.cos(angle/360*2*Math.PI))/angleConst; // PID Would be better, but this works for now.
            
            dist = Math.abs(angle) > Constants.AutoBalancing.max_pitch_offset ? dist : 0.0; // no speed if it is level
            
            double curPos = drivetrain.getPose().getY();
            
            double totalDistance = curPos+dist;

            double speed = controller.calculate(drivetrain.getPose().getY(),totalDistance);

            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speed, 0, 0, drivetrain.getRotation2d());

            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            drivetrain.setModuleStates(moduleStates);
        }


    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();

    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}