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
        // set motors speed to 0 just in case
        arm.setSpeedUpper(Constants.Arm.neo_motor_upper_stop);
        arm.setSpeedLower(Constants.Arm.neo_motor_lower_stop);
    }

    // Run every 20 ms
    @Override
    public void execute() {
        //Base.set(Constants.Base.speed.get().doubleValue());
        //neo motor speed isn't a constant yet
        //arm.setSpeedLower(Constants.Arm.neo_motor_lower_speed*-1);
    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        arm.setSpeedLower(Constants.Arm.neo_motor_lower_stop);
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}