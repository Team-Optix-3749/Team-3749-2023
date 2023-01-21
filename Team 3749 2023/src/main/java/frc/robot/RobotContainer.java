// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.testing.DrivetrainTesting;
import frc.robot.testing.MoveIndividualDrive;
import frc.robot.testing.MoveIndividualModule;
import frc.robot.testing.MoveIndividualTurning;
import frc.robot.utils.Constants;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;

public class RobotContainer {

    // Controllers
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);

    private final POV pilotPOV = new POV(pilot);
    private final POV operatorPOV = new POV(operator);

    

    // Subsystems
    // private final DrivetrainNew drivetrain = new DrivetrainNew();
    private final DrivetrainTesting drivetrain= new DrivetrainTesting();
    // Commands

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureDefaultCommands() {

        // regular
        // drivetrain.setDefaultCommand(new SwerveTeleopNew(drivetrain, pilot::getLeftX, pilot::getLeftY, pilot::getRightX,
        //         pilot.leftStick()::getAsBoolean));

        // Drive and
        // drivetrain.setDefaultCommand(new MoveIndividualModule(drivetrain, pilot::getLeftX, pilot::getLeftY, pilot::getRightX,
        //         pilot.leftStick()::getAsBoolean, Constants.SwerveENUMS.FRONT_LEFT));


        // drivetrain.setDefaultCommand(new MoveIndividualDrive(drivetrain, pilot::getLeftX, pilot::getLeftY, pilot::getRightX,
        //         pilot.leftStick()::getAsBoolean, Constants.SwerveENUMS.FRONT_LEFT));


        drivetrain.setDefaultCommand(new MoveIndividualTurning(drivetrain, pilot::getLeftX, pilot::getLeftY, pilot::getRightX,
                pilot.leftStick()::getAsBoolean,Constants.SwerveENUMS.FRONT_LEFT));
    }


    private void configureButtonBindings() {
        pilot.a().onTrue(new InstantCommand(drivetrain::toggleIdleMode));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
