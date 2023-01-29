// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCommand;

public class RobotContainer {

  // Controllers
  private final Xbox pilot = new Xbox(OIConstants.kDriverControllerPort);
  private final Xbox operator = new Xbox(1);

  // private final POV pilotPOV = new POV(pilot);
  // private final POV operatorPOV = new POV(operator);
  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  
  // Subsystems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Arm armSubsystem = new Arm();
  
  // Commands

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();

      swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
              swerveSubsystem,
              () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
              () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      configureButtonBindings();
  }

  private void configureDefaultCommands() {}

  private void configureButtonBindings() {
    pilot.a().whileTrue(new InstantCommand(swerveSubsystem::zeroHeading));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }  
}