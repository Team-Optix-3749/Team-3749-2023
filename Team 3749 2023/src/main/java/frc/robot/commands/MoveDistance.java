// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

    SwerveSubsystem swerveSubsystem;

    double setpoint;
    double dist_traveled = 0;
    double start_point;

    PIDController driveController = new PIDController(0.5, 0, 0);
    PIDController turnController = new PIDController(0.5, 0, 0);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);


    /***
     * 
     * @param swerveSubsystem the subsystem
     * @param dist       how far to to move in meters
     */
    public MoveDistance(SwerveSubsystem swerveSubsystem, double dist) {
        this.swerveSubsystem = swerveSubsystem;
        this.setpoint = dist;
        // start_point
        addRequirements(swerveSubsystem);
    }

    // Run on command init
    @Override
    public void initialize() {
        swerveSubsystem.stopModules();
        start_point = swerveSubsystem.getPose().getY();
    }

    // Run every 20 ms
    @Override
    public void execute() {

        // How inaccurate we are willing to be in reference to looking straight forward
        if (Math.abs(swerveSubsystem.getHeading()) > Constants.AutoBalancing.max_yaw_offset) {
            // negative so that we move towards the target, not away
            double turningSpeed = - turnController.calculate(Math.abs(swerveSubsystem.getHeading()),0);
            turningSpeed = turningLimiter.calculate(turningSpeed)
                    * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;

            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    0, 0, turningSpeed, swerveSubsystem.getRotation2d());
            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates =Constants. DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // 6. Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        } else {

            // where we are
            double current_position = swerveSubsystem.getPose().getY();
            // where we want to be
            // how fast to move
            double speed = driveController.calculate(current_position, start_point + setpoint);
            // and we move
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    0, speed, 0, swerveSubsystem.getRotation2d());
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);

            swerveSubsystem.setModuleStates(moduleStates);
        }

    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();

    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}