// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;

public class RobotContainer {

  // Controllers
  private final Xbox pilot = new Xbox(0);
  private final Xbox operator = new Xbox(1);

  private final POV pilotPOV = new POV(pilot);
  private final POV operatorPOV = new POV(operator);

  // Subsystems

  // Commands

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {}

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }  
}
