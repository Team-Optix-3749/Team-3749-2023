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
public class ClawCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    Claw claw = new Claw();
    
    

    Claw claw = new Claw();

    // Initializes the ClawCommand
    public ClawCommand(Claw claw) {
        this.claw = claw;
        addRequirements(claw);

    }
    public void IntakeOuttake(Claw claw, boolean Intake) {
        // false means outtake, true means intake
        double speed = 0;
        // speed tbd
        if (Intake == false) {
            claw.setSpeed(-speed, -speed);
        }
        else {
            claw.setSpeed(speed, speed);
        }
    }
    // Run on command init
    @Override
    public void initialize() {
    }

    // Run every 20 ms
    @Override
    public void execute() {
        // claw.set(Constants.claw.speed.get().doubleValue()); //set speed
    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}