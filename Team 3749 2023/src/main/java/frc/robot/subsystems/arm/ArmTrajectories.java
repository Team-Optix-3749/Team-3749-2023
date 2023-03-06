package frc.robot.subsystems.arm;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.utils.Constants;

/**
 * Generate arm trajectories
 * 
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 **/
public class ArmTrajectories {

    /**
     * Create trajectory
     * 
     * @param waypoints
     * @return trajectory
     */
    private static Trajectory createTrajectory(Pose2d[] waypoints, boolean isReversed) {
        if (isReversed) {
            // iterate through waypoints from last to first
            Collections.reverse(Arrays.asList(waypoints));

            // reverse pose for each waypoint (subtract pi)
            for (int i = 0; i < waypoints.length; i++)
                waypoints[i] = waypoints[i].transformBy(
                        new Transform2d(
                                new Translation2d(0.0, 0.0),
                                new Rotation2d(Math.PI)));
        }

        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(
                List.of(waypoints),
                new TrajectoryConfig(
                        Constants.Arm.maxSpeedMPS,
                        Constants.Arm.maxAccelerationMPS));
    }

    /**
     * Move arm to and from sting position and stow position
     * 
     * @param isReversed
     * @return Trajectory
     */
    public static Trajectory getStingTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.3, -0.2, new Rotation2d(Math.PI / 4)),
                new Pose2d(0.5, 0.7, new Rotation2d(Math.PI / 2)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from sting position and top node scoring position
     * 
     * @param isReversed
     * @return Trajectory
     */
    public static Trajectory getTopNodeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.5, 0.7, new Rotation2d(Math.PI / 2.5)),
                new Pose2d(1.1, 0.95, new Rotation2d(0)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from sting position and mid node scoring position
     * 
     * @param isReversed
     * @return Trajectory
     */
    public static Trajectory getMidNodeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.5, 0.7, new Rotation2d(0)),
                new Pose2d(1.05, 0.7, new Rotation2d(0)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    public static Trajectory getMidNodePlaceDownTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(1.05, 0.7, new Rotation2d(3 * Math.PI / 2)),
                new Pose2d(1.05, 0.5, new Rotation2d(3 * Math.PI / 2)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    public static Trajectory getMidNodePlaceReturnTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(1.05, 0.5, new Rotation2d(5 * Math.PI / 6)),
                new Pose2d(0.5, 0.7, new Rotation2d(5 * Math.PI / 6)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from stow position and double substiation loading position
     * 
     * @param isReversed
     * @return Trajectory
     */
    public static Trajectory getDoubleSubstationTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.3, -0.2, new Rotation2d(Math.PI / 4)),
                new Pose2d(0.5, 0.75, new Rotation2d(Math.PI / 2)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from stow position and further ground intake position
     * 
     * @param isReversed
     * @return Trajectory
     */
    public static Trajectory getGroundIntakeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.3, -0.2, new Rotation2d(0)),
                new Pose2d(0.7, -0.4, new Rotation2d(5 * Math.PI / 3)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from sting position and ground intake position
     * 
     * @param isReversed
     * @return Trajectory
     */
    public static Trajectory getGroundIntakeSweepTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.3, -0.2, new Rotation2d(0)),
                new Pose2d(1.4, -0.3, new Rotation2d(0)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    public static Trajectory getMidNodeToTopNodeTrajectory(boolean isReversed){
        Pose2d[] waypoints = new Pose2d[]{
            new Pose2d(1.05,0.7, new Rotation2d(Math.PI / 3)),
            new Pose2d(1.4,1.0, new Rotation2d(Math.PI / 8)),

        };

        return createTrajectory(waypoints, isReversed);
    }
}