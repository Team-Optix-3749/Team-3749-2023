// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.ColorSensor;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Harkirat Hattar
 * @author Aashray Reddy
 * 
 *         Command for the auto function of the claw
 * 
 * 
 * 
 * 
 * 
 *         extra space wow look here rohin does this anger you?
 */
public class ClawAutoCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    Claw claw;
    ColorSensor colorSensor; //static won't work w/o class instance; new instance of class required

    // Initializes the ClawAutoCommand
    public ClawAutoCommand(Claw Claw) {
        this.claw = Claw;
        addRequirements(Claw);
    }

    // Run on command init
    @Override
    public void initialize() {
    }

    // Run every 20 ms
    @Override
    public void execute() {
        colorSensor.ColorBasedRunning();
    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        // claw.setSpeed(Constants.Claw.stop); //set speed to 0 (stop)
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}