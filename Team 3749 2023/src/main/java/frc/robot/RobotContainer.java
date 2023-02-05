// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/***
 * @author Johnathan Liu
 * @author Aditya Samavedam
 * @author Don Tran
 */
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmSimulationCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;

public class RobotContainer {
  private final Xbox pilot = new Xbox(0);
  private final Xbox operator = new Xbox(1);

  private final POV pilotPOV = new POV(pilot);
  private final POV operatorPOV = new POV(operator);

  // Subsystems
  private final ArmSim armSim = new ArmSim();
  private final Arm arm = new Arm();

  // Commands

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {}

  // set as whileTrue, what are we going to do about timing, how long do we let it run continuously
  private void configureButtonBindings() { 
    // set as if statements dk if this works, might want to make separate function for this
    // TODO: need to implement enum (defaults are all zero for now)
    if (pilot.rightBumper().getAsBoolean()){ // cone nodes and single sub station
      if(pilot.x().getAsBoolean()){ // mid cone node
        pilot.x().whileTrue(new ArmCommand(arm, 0, 0));
      } 

      if(pilot.y().getAsBoolean()){ // high cone node
        pilot.y().whileTrue(new ArmCommand(arm, 0, 0));
      }

      if(pilot.a().getAsBoolean()){ // single sub
        pilot.a().whileTrue(new ArmCommand(arm, 0, 0));
      }
    }
    if (pilot.leftBumper().getAsBoolean()){ // cube nodes and double sub station
      if(pilot.x().getAsBoolean()){ // mid cube node
        pilot.x().whileTrue(new ArmCommand(arm, 0, 0));
      } 

      if(pilot.y().getAsBoolean()){ // high cube node
        pilot.y().whileTrue(new ArmCommand(arm, 0, 0));
      }

      if(pilot.a().getAsBoolean()){ // double sub
        pilot.a().whileTrue(new ArmCommand(arm, 0, 0));
      }
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }  
}
