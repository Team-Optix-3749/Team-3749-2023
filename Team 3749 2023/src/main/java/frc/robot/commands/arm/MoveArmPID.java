package frc.robot.commands.arm;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveArmPID extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final Translation2d position;
    private final Timer timer = new Timer();
    private final double duration;

    public MoveArmPID(Arm arm, Translation2d pos, double duration) {
        this.arm = arm;
        this.position = pos;
        this.duration = duration;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        try {
            arm.setArmPosition(position);
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= duration;
    }
}
