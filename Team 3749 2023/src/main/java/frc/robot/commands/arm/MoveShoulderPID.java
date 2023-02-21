package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveShoulderPID extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final double shoudlerSetpoint;

    public MoveShoulderPID(Arm arm, double shoudlerSetpoint) {
        this.arm = arm;
        this.shoudlerSetpoint = shoudlerSetpoint;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setShoulderAngle(shoudlerSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return arm.getShoulderAtSetpoint();
    }
}
