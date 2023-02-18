package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveElbowPID extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final double elevatorSetpoint;

    public MoveElbowPID(Arm arm, double elevatorSetpoint) {
        this.arm = arm;
        this.elevatorSetpoint = elevatorSetpoint;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setElbowAngle(elevatorSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return arm.isElbowAtSetpoint();
    }
}
