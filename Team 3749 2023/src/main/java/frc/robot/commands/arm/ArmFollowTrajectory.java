package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.arm.Arm;

public class ArmFollowTrajectory extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final Translation2d[] waypoints;
    
    private int current_waypoint_index = 0;
    private Translation2d current_waypoint;

    public ArmFollowTrajectory(Arm arm, Translation2d[] waypoints) {
        this.arm = arm;
        this.waypoints = waypoints;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        current_waypoint_index = 0;
        current_waypoint = waypoints[0];
    }

    @Override
    public void execute() {

        // will stay at the same waypoint if it is the last one
        if (current_waypoint_index != waypoints.length-1){
            // if the arm is sufficiantly close, go to the next waypoint
            if (Constants.withinMargin(0.25,  arm.getArmCoordinate(), current_waypoint)){
                current_waypoint_index +=1;
                current_waypoint = waypoints[current_waypoint_index];
            }
        }

        arm.setArmPosition(current_waypoint);
        SmartDashboard.putNumber("CURRENT WAYPOINT X", current_waypoint.getX());
        SmartDashboard.putNumber("CURRENT WAYPOINT Y", current_waypoint.getY());

        SmartDashboard.putNumber("Arm Coordinate X", arm.getArmCoordinate().getX());
        SmartDashboard.putNumber("Arm Coordinate Y", arm.getArmCoordinate().getY());
        SmartDashboard.putBoolean("MARGIN", Constants.withinMargin(0.1,  arm.getArmCoordinate(), current_waypoint));
        SmartDashboard.putBoolean("INDEX", current_waypoint_index != waypoints.length-1);

        // arm.setArmPosition(new Translation2d(0.75, 0.6));
        // SmartDashboard.putNumberArray("Trajectory Pos", new double[] {desiredState.poseMeters.getTranslation().getX(),desiredState.poseMeters.getTranslation().getY()});
        // System.out.println(desiredState.poseMeters.getTranslation().getY());

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
