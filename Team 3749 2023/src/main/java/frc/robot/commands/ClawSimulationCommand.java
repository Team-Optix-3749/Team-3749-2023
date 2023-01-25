// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawSimulationCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    ClawSim clawSim = new ClawSim();

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
        clawSim.set(Constants.Base.speed.get().doubleValue());
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