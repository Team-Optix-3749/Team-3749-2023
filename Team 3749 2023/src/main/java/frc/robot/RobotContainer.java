// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/***
 * @author Johnathan Liu
 * @author Aditya Samavedam
 * 
 */
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmSimulationCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;

public class RobotContainer {

  // Controllers
  // private final Arm arm = new Arm();

  // Replace with 
  // private final MoveLowerUp MoveLowerUpCommand = new MoveLowerUp(arm);
  // private final MoveLowerDown MoveLowerDownCommand = new MoveLowerDown(arm);
  // private final MoveUpperUpDown MoveUpperUpDownCommand = new MoveUpperUpDown(arm);
  
  // private final POV pilotPOV = new POV(pilot);
  // private final POV operatorPOV = new POV(operator);
  XboxController xbox = new XboxController(0);

  // Subsystems
  private final ArmSim armSim = new ArmSim();

  // Commands

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {}

  private void configureButtonBindings()
  {
    // JoystickButton A = new JoystickButton(xbox, Button.kA.value);
    // A.toggleOnTrue(new ArmSimulationCommand(armSim)); 
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }  
}
