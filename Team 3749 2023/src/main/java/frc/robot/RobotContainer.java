// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ResetTurning;
import frc.robot.commands.SwerveJoystickCommand;

public class RobotContainer {

  // Controllers
  // private final Xbox pilot = new Xbox(0);
  // private final Xbox operator = new Xbox(1);

  // private final POV pilotPOV = new POV(pilot);
  // private final POV operatorPOV = new POV(operator);

  private final XboxController xbox = new XboxController(0);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // private final Xbox pilot = new Xbox(OIConstants.kDriverControllerPort);

  // Subsystems

  // Commands
  ResetTurning resetTurning = new ResetTurning(swerveSubsystem);


  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
        swerveSubsystem,
        () -> -xbox.getLeftX(),
        () -> xbox.getLeftY(),
        () -> xbox.getRightX(),
        () -> !xbox.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    configureButtonBindings();
  }

  private void configureDefaultCommands() {
  }

  private void configureButtonBindings() {
    JoystickButton a = new JoystickButton(xbox, Button.kA.value);


    a.whileTrue(resetTurning);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
