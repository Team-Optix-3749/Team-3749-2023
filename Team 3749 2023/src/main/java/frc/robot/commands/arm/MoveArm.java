package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmTrajectories;
import frc.robot.utils.Constants.Arm.ArmSetpoints;;

public class MoveArm extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final ArmSetpoints desiredSetpoint;
    private Trajectory trajectory;
    private State desiredState;
    private Timer timer = new Timer();

    public MoveArm(Arm arm, ArmSetpoints setpoint) {
        this.arm = arm;
        this.desiredSetpoint = setpoint;
        // this.trajectory = trajectory;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        trajectory = ArmTrajectories.findTrajectory(desiredSetpoint, arm);

        timer.reset();
        timer.start();
        // System.out.println("START TRAJECTORY");
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

        try {
            FileWriter myWriter = new FileWriter("data.csv", true);
            myWriter.write(String.valueOf(desiredState.poseMeters.getX()) + ','
                    + String.valueOf(desiredState.poseMeters.getY()) + '\n');
            myWriter.close();
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

        System.out.println(
                String.valueOf(desiredState.poseMeters.getX()) + ',' + String.valueOf(desiredState.poseMeters.getY()));
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
