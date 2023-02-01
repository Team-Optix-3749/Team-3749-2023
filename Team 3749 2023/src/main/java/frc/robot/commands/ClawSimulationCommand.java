// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/***
 * @author Zaddy Harkirat 
 * @author Anusha Khobare
 * @author Aashray Reddy
 * @author Ryan R McWeeny
 * @author Hanlun Li
 *     Simulation code for the claw
 */

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawSimulationCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    ClawSim clawSim = new ClawSim();

    // left and right side of the claw (the motor)
    private CANSparkMax right_motor = new CANSparkMax(Constants.Claw.right_side, MotorType.kBrushless);
    private CANSparkMax left_motor = new CANSparkMax(Constants.Claw.left_side, MotorType.kBrushless);

    // motor controller group for both sides
    private MotorControllerGroup clawMotors = new MotorControllerGroup(left_motor, right_motor);

    // relative encoder
    private final RelativeEncoder claw_encoder = right_motor.getEncoder();

    // Initializes the ClawSimulationCommand
    public ClawSimulationCommand(ClawSim clawSim) {
        this.clawSim = clawSim;
        addRequirements(clawSim);
    }

    // Run on command init
    @Override
    public void initialize() {
    }

    // Run every 20 ms
    @Override
    public void execute() {
        clawSim.setSpeed(Constants.Base.speed.get().doubleValue());
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