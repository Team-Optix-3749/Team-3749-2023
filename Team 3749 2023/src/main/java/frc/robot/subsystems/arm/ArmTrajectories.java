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
                new Pose2d(0.8, 0.7, new Rotation2d(5 * Math.PI / 3)),
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
                new Pose2d(0.9, -0.09, new Rotation2d(5 * Math.PI / 3)),
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

    /**
     * Move arm from mid node to top node position
     * 
     * @param isReversed
     * @return Trajectory
     */
    public static Trajectory getMidNodeToTopNodeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.8, 0.7, new Rotation2d(Math.PI / 3)),
                new Pose2d(1.1, 0.95, new Rotation2d(Math.PI / 8)),

        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * information on an arm path detailing the multiple trajectories, where and for
     * how long to pause, and what the claw voltage should be during the trajectory.
     * Pause times take effect before the trajectory in its identical index
     * 
     * @param isReversed
     * @return Trajectory
     */
    public static enum ArmPaths {
        STOW_TO_TOP(
                new Trajectory[] { getTopNodeTrajectory(false) }, // trajectories
                new double[] { 0 }, // pause lengths
                new double[] { Constants.Claw.idleVoltage }), // voltages
        STING_TO_TOP(
                new Trajectory[] { getStingTrajectory(false).concatenate(getTopNodeTrajectory(false)) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        TOP_TO_STOW(
                new Trajectory[] {
                        getTopNodeTrajectory(true).concatenate(
                                getStingTrajectory(true))
                },
                new double[] { 0, 0.4 },
                new double[] { Constants.Claw.idleVoltage, Constants.Claw.releaseObjectVoltage }),
        TOP_TO_STING(
                new Trajectory[] { getTopNodeTrajectory(true) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        TOP_TO_MID(
                new Trajectory[] { getMidNodeToTopNodeTrajectory(true) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        STOW_TO_MID(
                new Trajectory[] { getStingTrajectory(false).concatenate(getMidNodeTrajectory(false)) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        STING_TO_MID(
                new Trajectory[] { getMidNodeTrajectory(false) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        MID_TO_STOW(
                new Trajectory[] {
                        ArmTrajectories.getMidNodeTrajectory(true).concatenate(
                                ArmTrajectories.getStingTrajectory(true))
                },
                new double[] { 0, 0.4 },
                new double[] { Constants.Claw.idleVoltage, Constants.Claw.releaseObjectVoltage }),
        MID_TO_STING(
                new Trajectory[] { getMidNodeTrajectory(true) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        MID_TO_TOP(
                new Trajectory[] { getMidNodeToTopNodeTrajectory(false) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        STOW_TO_DOUBLESUB(
                new Trajectory[] { getDoubleSubstationTrajectory(false) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        DOUBLESUB_TO_STOW(
                new Trajectory[] { getDoubleSubstationTrajectory(true) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        STOW_TO_GROUND_INTAKE(
                new Trajectory[] { getGroundIntakeTrajectory(false) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        GROUND_INTAKE_TO_STOW(
                new Trajectory[] { getGroundIntakeTrajectory(true) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        STOW_TO_STING(
                new Trajectory[] { getStingTrajectory(false) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage }),
        STING_TO_STOW(
                new Trajectory[] { getStingTrajectory(true) },
                new double[] { 0 },
                new double[] { Constants.Claw.idleVoltage });

        public int numTrajectories;
        public Trajectory[] trajectories;
        public double[] trajectoryLengths;
        public double[] pauseLengths;
        public double[] clawVoltages;

        ArmPaths(Trajectory[] trajectories,
                double[] pauseLengths,
                double[] clawVoltages) {
            this.numTrajectories = trajectories.length;
            this.trajectoryLengths = new double[numTrajectories];
            int index = 0;
            for (Trajectory traj : trajectories) {
                this.trajectoryLengths[index] = traj.getTotalTimeSeconds();
                index++;
            }

            this.trajectories = trajectories;
            this.pauseLengths = pauseLengths;
            this.clawVoltages = clawVoltages;
        }

    }

}