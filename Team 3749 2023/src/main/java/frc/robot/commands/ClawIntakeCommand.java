// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Rohin Sood
 * 
 *         Serves as a template to format commands
 */
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
        //claw.setSpeed(-Constants.speed, -Constants.speed);
    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        claw.setSpeed(Constants.Claw.stop);
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}