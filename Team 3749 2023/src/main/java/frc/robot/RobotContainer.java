// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    final Xbox pilot = new Xbox(0);
    final Xbox operator = new Xbox(1);

    final POV pilotPOV = new POV(pilot);
    final POV operatorPOV = new POV(operator);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
