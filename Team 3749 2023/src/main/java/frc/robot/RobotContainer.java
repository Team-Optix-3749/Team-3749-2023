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
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

// Replace with ArmCommand.java later
// import frc.robot.commands.MoveUpperUpDown;
// import frc.robot.commands.MoveLowerDown;
// import frc.robot.commands.MoveLowerUp;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;

public class RobotContainer {

  // Controllers
  private final Arm arm = new Arm();

  // Replace with 
  // private final MoveLowerUp MoveLowerUpCommand = new MoveLowerUp(arm);
  // private final MoveLowerDown MoveLowerDownCommand = new MoveLowerDown(arm);
  // private final MoveUpperUpDown MoveUpperUpDownCommand = new MoveUpperUpDown(arm);
  
  private final Xbox pilot = new Xbox(0);
  private final Xbox operator = new Xbox(1);
  private final POV pilotPOV = new POV(pilot);
  private final POV operatorPOV = new POV(operator);
  private final XboxController xboxController = new XboxController(0);

  // joystick
  private final Joystick joystick = new Joystick(Constants.Controller.joystick_port);

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

    // Replace with commands for ArmCommand.java
    // pilotPOV.up().toggleOnTrue(MoveLowerUpCommand); //if up button on Dpad is pressed, run MoveLowerUpCommand
    // pilotPOV.down().toggleOnTrue(MoveLowerDownCommand); //if down button on Dpad is pressed, run MoveLowerDownCommand 
    // pilot.x().toggleOnTrue(MoveUpperUpDownCommand); //if x button is pressed, run MoveUpperUpDownCommand
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }  
}
