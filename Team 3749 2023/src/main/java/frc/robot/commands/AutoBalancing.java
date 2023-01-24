// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Noah Simon
 * 
 *         Allows the robot to automatically "engage" on the charging station
 */
public class AutoBalancing extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    DrivetrainNew drivetrain;

    double angle = drivetrain.getVerticalTilt();

    // Initializes the BaseCommand
    public AutoBalancing(DrivetrainNew drivetrain) {
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
            drivetrain.turnToZeroHeading();

        }
        else{
            angle = drivetrain.getVerticalTilt();
            double speed =  angle / 100; // PID Would be better, but this works for now.
            speed = Math.abs(angle) > Constants.AutoBalancing.max_pitch_offset ? speed : 0.0; // no speed if it is level
    
            // Set chassis speed to be only forward, relative to the field.
            ChassisSpeeds chassisSpeeds;
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( 0, speed, 0, drivetrain.getRotation2d());
            SwerveModuleState[] states = Constants.DrivetrainNew.kinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(states);
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