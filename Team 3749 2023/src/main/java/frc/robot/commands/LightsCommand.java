// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Jonathan Liu
 * @author Bailey Say
 *         
 */
public class LightsCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    Lights lights = new Lights();
    Timer timer = new Timer();

    // Settings is basically a variable alternating between true and false to allow each LED to change from color to color
    boolean settings = true;

    // Initializes the BaseCommand
    public LightsCommand(Lights lights) {
    
        timer.start();
    }

    // Run on command init
    @Override
    public void initialize() {
    }

    
    @Override
    public void execute() {

        // Checks if in the first 1/50 of a second to change color
        if (timer.get() % 1000 <= 20) {
            if (settings) {
                lights.setRGBOfStrip(255, 255, 255, 0, 255, 0);
                settings = false;
            } else {
                lights.setRGBOfStrip(0, 255, 0, 255, 255, 255);
                settings = true;
            }
        }

        // Make sure timer.get() value isn't too big
        if (timer.get() >= 10000) {
            timer.reset();
        }
    }
     

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}