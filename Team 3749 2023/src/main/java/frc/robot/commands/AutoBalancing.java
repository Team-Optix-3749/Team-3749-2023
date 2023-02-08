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
 */
public class AutoBalancing extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    SwerveSubsystem swerveSubsystem;

    double angle = swerveSubsystem.getVerticalTilt();
    PIDController controller = new PIDController(0.5, 0, 0);
    double start_position;
    double position;
    double end_position;

    // Initializes the BaseCommand
    public AutoBalancing(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        start_position = swerveSubsystem.getPose().getY();
        position = swerveSubsystem.getPose().getY();
        addRequirements(swerveSubsystem);
    }

    // Run on command init
    @Override
    public void initialize() {
        swerveSubsystem.stopModules();
        // update start position when we are getting started
        start_position = swerveSubsystem.getPose().getY();

        angle = swerveSubsystem.getVerticalTilt();
        // Math lookin ass to get the distance on the station we need to travel based on
        // the angle see https://www.desmos.com/calculator/ujt90ivwwc for the desmos of
        // the function
        double dist = calculateDistance(angle);
        end_position = start_position + dist;
    }

    // Run every 20 ms
    @Override
    public void execute() {
        // How inaccurate we are willing to be in reference to looking straight forward
        if (Math.abs(swerveSubsystem.getHeading()) > Constants.AutoBalancing.max_yaw_offset) {
            // NOT CORRECT, see MoveDistance
            // SwerveModuleState[] states = new SwerveModuleState[4];
            // for (int i = 0; i < 4; i++) {
            // states[i] = new SwerveModuleState(0.0, new Rotation2d(0));
            // }
            // swerveSubsystem.setModuleStates(states);
        }
        // how close to the setpoint we want to be before calculating a new one
        if (Math.abs(controller.getPositionError()) < Constants.AutoBalancing.max_movement_offset) {
            start_position = swerveSubsystem.getPose().getY();
            angle = swerveSubsystem.getVerticalTilt();
            // Math lookin ass to get the distance on the station we need to travel based on
            // the angle see https://www.desmos.com/calculator/ujt90ivwwc for the desmos of
            // the function
            double dist = calculateDistance(angle);
            end_position = start_position + dist;
        } else {
            // Tilt of the robot
            position = swerveSubsystem.getPose().getY();
            // how fast to move
            double speed = controller.calculate(position, end_position);
            // ...and we move
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    0, speed, 0, swerveSubsystem.getRotation2d());
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);
            for (int i = 0; i < 4; i++) {
                moduleStates[i] = new SwerveModuleState(moduleStates[i].speedMetersPerSecond, new Rotation2d(0));
            }
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

    /***
     * 
     * @param angle the gyro read angle
     * @return distance, how far to travel on the charge station in meters
     */
    private double calculateDistance(double angle) {
        double dist = 0;
        if (Math.abs(angle) > Constants.AutoBalancing.max_pitch_offset) {
            // value at the max degree amount,
            double angleConst = Math.cos(15 / 360 * 2 * Math.PI);
            // The distance left to travel over the floor, or adjacent side of the triangle
            double distAdjacent = (1 - Math.cos(angle / 360 * 2 * Math.PI)) / (1 - angleConst);
            // Ratio of the adjacent side to the hypotenuse, which is the actualy platform
            // length we care about
            double adjacent_to_hypotenuse = Math.cos(angle / 360 * 2 * Math.PI) / angleConst;
            // calculates the hypotonuse distance, then multiplies by 2ft, the specified
            // midpoint of the platform
            dist = Units.feetToMeters(distAdjacent / adjacent_to_hypotenuse * 2);

        }
        // reapplies the sign of the angle to offset the absolute value
        if (angle < 0) {
            dist *= -1;
        }
        return dist;
    }
}