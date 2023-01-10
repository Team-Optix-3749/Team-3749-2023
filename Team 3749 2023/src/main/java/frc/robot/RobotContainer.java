// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveTeleopNew;
import frc.robot.commands.SwerveTeleopOld;
import frc.robot.subsystems.DrivetrainOld;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;

public class RobotContainer {

  // Controllers
  private final Xbox pilot = new Xbox(0);
  private final Xbox operator = new Xbox(1);

  private final POV pilotPOV = new POV(pilot);
  private final POV operatorPOV = new POV(operator);


  // Subsystems
  private final DrivetrainOld drivetrain = new DrivetrainOld();
  // private final DrivetraiNew drivetrain = new DrivetrainNew();

  // Commands

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new SwerveTeleopOld(drivetrain, pilot::getLeftY, pilot::getLeftY,  pilot::getRightX , pilot.leftStick()::getAsBoolean));
    // drivetrain.setDefaultCommand(new SwerveTeleopNew(drivetrain, pilot::getLeftY, pilot::getLeftY,  pilot::getRightX , pilot.leftStick()::getAsBoolean));
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }  
}
