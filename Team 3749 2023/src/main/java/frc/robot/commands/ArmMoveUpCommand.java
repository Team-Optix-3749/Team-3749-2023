// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Aditya Samavedam
 * @author Don Tran
 * 
 */
public class ArmMoveUpCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    Arm arm = new Arm();

    // Initializes the ArmCommand
    public ArmMoveUpCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // Run on command init
    @Override
    public void initialize() {
        // set motors speed to 0 just in case
        arm.setSpeedTelescope(0);
        arm.setSpeedElevator(0);
    }  

    // Run every 20 ms
    @Override
    public void execute() {
        //Base.set(Constants.Base.speed.get().doubleValue());
        arm.setSpeedElevator(Constants.Arm.neo_motor_speed);
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