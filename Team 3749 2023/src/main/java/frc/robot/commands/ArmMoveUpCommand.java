// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;

import java.sql.Time;

import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Aditya Samavedam
 * @author Don Tran
 * 
 */
public class ArmMoveUpCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private Arm arm = new Arm();

    // Initializes the ArmCommand
    public ArmMoveUpCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // Run on command init
    @Override
    public void initialize() {
        // set motors speed to 0 just in case
        arm.setSpeedElevator(Constants.Arm.neo_motor_elevator_speed);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            end(isFinished());
        }
    }

    // Run every 20 ms
    @Override
    public void execute() {
        // Base.set(Constants.Base.speed.get().doubleValue());
    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        arm.setSpeedElevator(Constants.Arm.neo_motor_elevator_stop);
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}