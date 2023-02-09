// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    String message;
  
    /***
     * 
     * @param swerveSubsystem the subsystem
     * @param dist       how far to to move in meters
     */
    public MoveDistance(String message) {
        this.message = message;
        // start_point
        addRequirements(swerveSubsystem);
    }

    // Run on command init
    @Override
    public void initialize() {
        System.out.println(message);

    }

    // Run every 20 ms
    @Override
    public void execute() {


    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}