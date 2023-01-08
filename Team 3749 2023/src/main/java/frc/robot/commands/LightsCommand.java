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
 * 
 *         
 */
public class LightsCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    Lights lights = new Lights();
    Timer timer = new Timer();

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
        if (timer.get() % 1000 <= 20) {
            lights.setRGBOfStrip(255, 255, 255, 0, 255, 0);
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