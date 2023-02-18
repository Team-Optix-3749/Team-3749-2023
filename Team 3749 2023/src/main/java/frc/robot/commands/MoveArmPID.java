package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveArmPID extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final double shoulderSetpoint;
    private final double elevatorSetpoint;

    public MoveArmPID(Arm arm, double shoulderSetpoint, double elevatorSetpoint) {
        this.arm = arm;
        this.shoulderSetpoint = shoulderSetpoint;
        this.elevatorSetpoint = elevatorSetpoint;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setShoulderAngle(shoulderSetpoint);
        arm.setElbowAngle(elevatorSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtSetpoint() && arm.isElbowAtSetpoint();
    }
}
