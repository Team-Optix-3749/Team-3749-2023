// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ArmExtendRetractCommand;
import frc.robot.commands.ArmMoveDownCommand;
import frc.robot.commands.ArmMoveUpCommand;
import frc.robot.subsystems.Arm;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;

public class RobotContainer {

  // Controllers
  private final Arm arm = new Arm();
  private final ArmMoveUpCommand armMoveUpCommand = new ArmMoveUpCommand(arm);
  private final ArmMoveDownCommand armMoveDownCommand = new ArmMoveDownCommand(arm);
  private final ArmExtendRetractCommand armExtendRetractCommand = new ArmExtendRetractCommand(arm);
  
  private final Xbox pilot = new Xbox(0);
  private final Xbox operator = new Xbox(1);
  private final POV pilotPOV = new POV(pilot);
  private final POV operatorPOV = new POV(operator);
  private final XboxController joystick = new XboxController(0);

  // Subsystems

  // Commands

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {}

  private void configureButtonBindings()
  {
    /*POV pov = new POV(operator);
    POVButton upButton = pov.up();
    POVButton downButton = pov.down();*/
    pilotPOV.up().toggleOnTrue(armMoveUpCommand);
    pilotPOV.down().toggleOnTrue(armMoveDownCommand);
    pilot.x().toggleOnTrue(armExtendRetractCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }  
}
