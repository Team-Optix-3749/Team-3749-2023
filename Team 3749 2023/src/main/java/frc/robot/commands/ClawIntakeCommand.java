// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/***
 * @author Anusha Khobare
 * @author Aashray Reddy
 * @author Ryan R McWeeny
 * @author Hanlun Li
 * 
 *     ClawOuttake.Java is a comand that runs the claw motors backwards causing it to intake objects (dependent on Claw.Java and Constants.java)
 */

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ClawIntakeCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    Claw claw;

    // Initializes the ClawCommand
    public ClawIntakeCommand(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    // Run on command init
    @Override
    public void initialize() {
    }

    // Run every 20 ms
    @Override
    public void execute() {
        // uses PID to calculate the velocity needed to acheive an exact speed
        claw.setSpeed(-Constants.Claw.setpoint);
        
    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        claw.setSpeed(Constants.Claw.stop); //set speed to 0 (stop)
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}