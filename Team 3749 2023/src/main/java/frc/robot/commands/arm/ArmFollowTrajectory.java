package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmFollowTrajectory extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final Trajectory trajectory;
    private State desiredState;
    private Timer timer = new Timer();

    public ArmFollowTrajectory(Arm arm, Trajectory trajectory) {
        this.arm = arm;
        this.trajectory = trajectory;

        addRequirements(arm);
    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {


        double cur_time = timer.get();
        desiredState = trajectory.sample(cur_time);

        try {
            arm.setArmPosition(desiredState.poseMeters.getTranslation());
        } catch (Exception e) {
            System.out.println(e);
        }
        System.out.println(desiredState.poseMeters.getY());
        logging();

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return trajectory.getTotalTimeSeconds() < timer.get();
    }

    public void logging() {
        SmartDashboard.putNumber("CURRENT WAYPOINT X", desiredState.poseMeters.getTranslation().getX());
        SmartDashboard.putNumber("CURRENT WAYPOINT Y", desiredState.poseMeters.getTranslation().getY());

        SmartDashboard.putNumber("Arm Coordinate X", arm.getArmCoordinate().getX());
        SmartDashboard.putNumber("Arm Coordinate Y", arm.getArmCoordinate().getY());
    }
}
