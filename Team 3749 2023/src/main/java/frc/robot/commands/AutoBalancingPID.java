// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands;

import java.lang.ModuleLayer.Controller;

import javax.swing.plaf.nimbus.State;

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
 *         Allows the robot to automatically "engage" on the charging station
 *         Strategy: Move forward until change in angle. Then use PID from the
 *         angle measurement to balance
 */
public class AutoBalancingPID extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    SwerveSubsystem swerveSubsystem;

    private final PIDController controller = new PIDController(0.1, 0, 0);
    private final PIDController turnController = new PIDController(0.005, 0.001, 0);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    private double angle;
    private double prev_angle;
    private double heading;
    private boolean has_flipped;

    // Initializes the BaseCommand
    public AutoBalancingPID(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    // Run on command init
    @Override
    public void initialize() {
        swerveSubsystem.stopModules();
        // update start position when we are getting started
        prev_angle = swerveSubsystem.getVerticalTilt();
        angle = swerveSubsystem.getVerticalTilt();
        has_flipped = false;

    }

    // Run every 20 ms
    @Override
    public void execute() {
        heading = swerveSubsystem.getHeading();
        prev_angle = angle;
        angle = swerveSubsystem.getVerticalTilt();
        // How inaccurate we are willing to be in reference to looking straight forward
        // Should change this so it adjusts on the go and doesn't need to stop
        if (withinMargin(Constants.AutoBalancing.max_pitch_offset, heading, 0)) {
            // negative so that we move towards the target, not away
            double turning_speed = turnController.calculate(Math.abs(heading), 0);
            turning_speed = turningLimiter.calculate(turning_speed)
                    * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            // signs the speed so we move in the correct direction
            turning_speed = Math.abs(turning_speed) * Math.signum(heading);
            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    0, 0, turning_speed, swerveSubsystem.getRotation2d());
            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);
            // 6. Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        }

        // move forward if the angle hasn't started to move and it hasn't moved in the
        // past
        if (withinMargin(Constants.AutoBalancing.max_pitch_margin, prev_angle, angle) && !has_flipped) {
            // Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    Constants.AutoBalancing.base_speed_mps, 0, 0, swerveSubsystem.getRotation2d());
            // Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);
            // Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        }
        // the robot must've moved slightly past the center now, so we will start using
        // PID to reach the middle
        else if (!withinMargin(Constants.AutoBalancing.max_pitch_offset, angle, 0)) {
            has_flipped = true;
            double speed = controller.calculate(angle, 0)
                    * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            // not sure if this is necesary
            // // signs the speed so we move in the correct direction
            // speed = Math.abs(speed) * Math.signum(angle);

            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    speed, 0, 0, swerveSubsystem.getRotation2d());
            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);
            // 6. Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        }
        // we level? Stop with wheels facing different directions to prevent sliding
        else {
            SwerveModuleState[] states = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                states[i] = new SwerveModuleState(0, new Rotation2d(45 + 90 * i));
            }
            swerveSubsystem.setModuleStates(states);
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

    /***
     * 
     * @param margin how close the values need to be to return true. Use a positive
     *               number
     * @param a      the first number
     * @param b      the second number
     * @return true if it is within the margin, false if not
     */
    private boolean withinMargin(double margin, double a, double b) {
        if (a + margin >= b && a - margin <= b || a == b) {
            return true;
        }
        return false;
    }
}