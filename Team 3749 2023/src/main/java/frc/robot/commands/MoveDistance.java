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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Noah Simon
 * 
 *         Moves the robot a specific amount forwrad on a button press
 */
public class MoveDistance extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    SwerveSubsystem drivetrain;

    double setpoint;
    double dist_traveled = 0;
    double start_point;

    PIDController controller = new PIDController(0.5, 0, 0);

    /***
     * 
     * @param drivetrain the subsystem
    * @param dist       how far to to move in meters
     */
    public MoveDistance(SwerveSubsystem drivetrain, double dist) {
        this.drivetrain = drivetrain;
        this.setpoint = dist;
        // start_point
        addRequirements(drivetrain);
    }

    // Run on command init
    @Override
    public void initialize() {
        drivetrain.stopModules();
        start_point = drivetrain.getPose().getY();
    }

    // Run every 20 ms
    @Override
    public void execute() {

         // How inaccurate we are willing to be in reference to looking straight forward
         if (Math.abs(drivetrain.getHeading())>Constants.AutoBalancing.max_yaw_offset){
            SwerveModuleState[] states = new SwerveModuleState[4];
            for (int i = 0; i <4; i++){
                states[i] = new SwerveModuleState(0.0, new Rotation2d(0));
            }
            drivetrain.setModuleStates(states);
        }
        else{

            // where we are
            double current_position = drivetrain.getPose().getY();
            // where we want to be
            // how fast to move
            double speed = controller.calculate(current_position, start_point+setpoint);
            // and we move
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, speed, 0, drivetrain.getRotation2d());
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