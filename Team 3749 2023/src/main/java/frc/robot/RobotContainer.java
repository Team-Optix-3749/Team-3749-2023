// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SwerveTeleopNew;
import frc.robot.subsystems.DrivetrainNew;
import frc.robot.testing.DrivetrainTesting;
import frc.robot.testing.MoveIndividualModule;
import frc.robot.testing.TestEncoderValues;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;

public class RobotContainer {

    // Controllers
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);

    private final POV pilotPOV = new POV(pilot);
    private final POV operatorPOV = new POV(operator);

    private final SendableChooser<String> runModeChooser = new SendableChooser<>();

    // Subsystems
    // private final DrivetrainNew drivetrain = new DrivetrainNew();
    private DrivetrainTesting drivetrainTesting = null;
    private DrivetrainNew drivetrain = null;
    // Commands

    public RobotContainer() {
        runModeChooser.setDefaultOption("Default", "default");
        runModeChooser.addOption("Test", "test");
        String run_mode = runModeChooser.getSelected();
        if (run_mode != "test") {
            drivetrain = new DrivetrainNew();
            // regular
            drivetrain.setDefaultCommand(
                    new SwerveTeleopNew(drivetrain, pilot::getLeftX, pilot::getLeftY, pilot::getRightX,
                            pilot.leftStick()::getAsBoolean));
        } else {
            // TESTING
            drivetrainTesting = new DrivetrainTesting();
            drivetrainTesting.setDefaultCommand(
                    new MoveIndividualModule(drivetrainTesting, pilot::getLeftX, pilot::getLeftY, pilot::getRightX,
                            pilot.leftStick()::getAsBoolean));
        }

        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureDefaultCommands() {

    }

    private void configureButtonBindings() {
        String run_mode = runModeChooser.getSelected();
        if (run_mode == "test") {
            pilot.a().onTrue(new InstantCommand(drivetrainTesting::toggleIdleMode));
            pilot.b().whileTrue(new TestEncoderValues(drivetrainTesting));
        } else {
            pilot.a().onTrue(new InstantCommand(drivetrain::toggleIdleMode));
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
