// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Don Tran
 * @author Bailey Say
 * @author Raymond Sheng
 * 
 */
public class ArmCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private Arm arm;

    // Initializes the ArmCommand
    public ArmCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // Run on command init
    @Override
    public void initialize() {
        
    }

    // Run every 20 ms
    @Override
    public void execute() { // we will figure this out later
        
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