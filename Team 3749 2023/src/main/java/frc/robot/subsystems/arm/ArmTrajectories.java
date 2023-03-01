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

public class ArmTrajectories {

    /**
     * Create trajectory using Translation2d
     * 
     * @param waypoints
     * @return trajectory
     */
    private static Trajectory createTrajectory(Translation2d[] waypoints) {
        // start and end coordinates, taken from waypoints
        double[] startXY = new double[] { waypoints[0].getX(), waypoints[0].getY() };
        double[] endXY = new double[] { waypoints[waypoints.length - 1].getX(),
                waypoints[waypoints.length - 1].getY() };

        Translation2d[] midpoints = new Translation2d[waypoints.length - 2];
        for (int i = 1; i < waypoints.length - 1; i++) {
            midpoints[i - 1] = waypoints[i];
        }

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                Constants.Arm.maxSpeedMPS, Constants.Arm.maxAccelerationMPS);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(startXY[0], startXY[1], new Rotation2d(0)),
                List.of(midpoints),
                new Pose2d(endXY[0], endXY[1], new Rotation2d(0)),
                trajectoryConfig);

        return trajectory;
    }

    /**
     * Create trajectory using Pose2d
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

    public static Trajectory getTopNodeTrajectoryPose(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.3, -0.2, new Rotation2d(0)),
                new Pose2d(1.0, 1.0, new Rotation2d(0)),
                new Pose2d(1.4, 1.0, new Rotation2d(0)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    public static Trajectory getStingTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.3, -0.2, new Rotation2d(Math.PI / 4)),
                new Pose2d(0.5, 0.7, new Rotation2d(Math.PI / 2)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    public static Trajectory getTopNodeTrajectory(boolean reverse) {
        Translation2d[] waypoints = new Translation2d[] {
                new Translation2d(0.35, -0.2),
                new Translation2d(0.4, -0.175),
                new Translation2d(0.45, -0.05),
                new Translation2d(0.7, 0.8),
                new Translation2d(1.4, 1.0)
        };
        if (reverse) {
            Collections.reverse(Arrays.asList(waypoints));
        }
        return createTrajectory(waypoints);
    }

    public static Trajectory getMidNodeTrajectory(boolean reverse) {
        Translation2d[] waypoints = new Translation2d[] {
                new Translation2d(0.35, -0.2),
                new Translation2d(0.4, -0.175),
                new Translation2d(0.45, -0.05),
                new Translation2d(0.9, 0.75),
                new Translation2d(0.95, 0.75),
        };
        if (reverse) {
            Collections.reverse(Arrays.asList(waypoints));
        }
        return createTrajectory(waypoints);
    }

    public static Trajectory getDoubleSubstationTrajectory(boolean reverse) {
        Translation2d[] waypoints = new Translation2d[] {
                new Translation2d(0.35, -0.2),
                new Translation2d(0.4, -0.15),
                new Translation2d(0.45, 0),
                new Translation2d(0.6, 0.65),
                new Translation2d(0.7, 0.7),
                new Translation2d(0.75, 0.75) };
        if (reverse) {
            Collections.reverse(Arrays.asList(waypoints));
        }
        return createTrajectory(waypoints);
    }

    public static Trajectory getGroundPickupTrajectory(boolean reverse) {
        Translation2d[] waypoints = new Translation2d[] {
                new Translation2d(0.35, -0.2),
                new Translation2d(0.625, -0.2),
                new Translation2d(0.625, -0.35),
                new Translation2d(0.625, -0.425) };
        if (reverse) {
            Collections.reverse(Arrays.asList(waypoints));
        }
        return createTrajectory(waypoints);
    }
}
