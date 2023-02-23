package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.arm.Arm;

public class ArmFollowTrajectory extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final Trajectory trajectory;
    
    private Timer timer = new Timer();
    private Translation2d current_waypoint;

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
        State desiredState = trajectory.sample(cur_time);

        arm.setArmPosition(desiredState.poseMeters.getTranslation());
        SmartDashboard.putNumber("CURRENT WAYPOINT X", desiredState.poseMeters.getTranslation().getX());
        SmartDashboard.putNumber("CURRENT WAYPOINT Y", desiredState.poseMeters.getTranslation().getY());

        SmartDashboard.putNumber("Arm Coordinate X", arm.getArmCoordinate().getX());
        SmartDashboard.putNumber("Arm Coordinate Y", arm.getArmCoordinate().getY());


    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
