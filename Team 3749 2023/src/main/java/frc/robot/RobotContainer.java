// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.MoveDistance;
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
  private final MoveDistance moveDistance = new MoveDistance(swerveSubsystem, Units.feetToMeters(5));

  public RobotContainer() {
    setupAuto();
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

  private void configureDefaultCommands() {
  }

  private void configureButtonBindings() {
    pilot.a().whileTrue(new InstantCommand(swerveSubsystem::zeroHeading));
    pilot.b().whileTrue(moveDistance);

  }

  public Command getAutonomousCommand() {
    return AutoCommands.getTestPathPlanner(swerveSubsystem);
  }

  public void setupAuto(){
    Constants.AutoConstants.eventMap.put("pickup_cone_floor", new PrintCommand("PICKUP CONE FLOOR"));
    Constants.AutoConstants.eventMap.put("pickup_cube_floor", null);
    Constants.AutoConstants.eventMap.put("pickup_cone_double_substation", null);
    Constants.AutoConstants.eventMap.put("pickup_cube_double_substation", null);
    Constants.AutoConstants.eventMap.put("pickup_cone_single_substation", null);
    Constants.AutoConstants.eventMap.put("pickup_cube_single_substation", null);
    Constants.AutoConstants.eventMap.put("place_cone_bottom", null);
    Constants.AutoConstants.eventMap.put("place_cube_bottom", null);
    Constants.AutoConstants.eventMap.put("place_cone_mid", null);
    Constants.AutoConstants.eventMap.put("place_cube_mid", null);
    Constants.AutoConstants.eventMap.put("place_cone_top", null);
    Constants.AutoConstants.eventMap.put("place_cube_top", null);

  }
}
