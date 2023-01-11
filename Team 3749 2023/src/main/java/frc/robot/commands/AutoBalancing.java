// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Noah Simon
 * 
 *         Allows the robot to automatically "engage" on the charging station
 */
public class AutoBalancing extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    DrivetrainOld drivetrain = new DrivetrainOld();

    // Initializes the BaseCommand
    public AutoBalancing(DrivetrainOld drivetrain) {
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
        

        // SwerveModuleState[] states = Constants.DrivetrainOld.driveKinematics.toSwerveModuleStates(chassisSpeeds);
        // drivetrain.setModuleStates(states);
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