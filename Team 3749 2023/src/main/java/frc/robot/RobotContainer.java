// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/***
 * @author Johnathan Liu
 * @author Aditya Samavedam
 * @author Don Tran
 */
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.utils.Xbox;

public class RobotContainer {
  private final Xbox pilot = new Xbox(0);
  private final Xbox operator = new Xbox(1);

  // Subsystems
  private final Arm arm = new Arm();

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
  }

  // set as whileTrue, what are we going to do about timing, how long do we let it
  // run continuously
  private void configureButtonBindings() {
    pilot.aWhileHeld(
        () -> arm.setShoulder(.2), () -> arm.setShoulder(0), arm);
    pilot.aWhileHeld(
        () -> arm.setShoulder(-.2), () -> arm.setShoulder(0), arm);

    pilot.xWhileHeld(
        () -> arm.setElbow(.2), () -> arm.setElbow(0), arm);
    pilot.yWhileHeld(
        () -> arm.setElbow(-.2), () -> arm.setElbow(0), arm);

    pilot.rightBumperWhileHeld(
        () -> arm.setShoulderPosition(0.66), () -> arm.setShoulderPosition(0), arm);
    pilot.leftBumperWhileHeld(
        () -> arm.setShoulderPosition(0.45), () -> arm.setShoulderPosition(0), arm);
}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
