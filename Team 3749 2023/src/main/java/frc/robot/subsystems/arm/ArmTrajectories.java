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
import frc.robot.utils.Constants.Arm.ArmSetpoints;

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

    public static Trajectory getStingTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.3, -0.2, new Rotation2d(Math.PI / 4)),
                new Pose2d(0.5, 0.7, new Rotation2d(Math.PI / 2)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    public static Trajectory getTopNodeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.5, 0.7, new Rotation2d(Math.PI / 8)),
                new Pose2d(1.33, 1.0, new Rotation2d(Math.PI / 8)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    public static Trajectory getMidNodeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.5, 0.7, new Rotation2d(Math.PI / 3)),
                new Pose2d(0.9, 0.7, new Rotation2d(Math.PI / 3)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    public static Trajectory getMidNodeToTopNodTrajectory(boolean isReversed){
        Pose2d[] waypoints = new Pose2d[]{
            new Pose2d(0.9,0.7, new Rotation2d(Math.PI / 3)),
            new Pose2d(1.33,1.0, new Rotation2d(Math.PI / 8)),

        };

        return createTrajectory(waypoints, false);
    }

    public static Trajectory getDoubleSubstationTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.3, -0.2, new Rotation2d(Math.PI / 4)),
                new Pose2d(0.75, 0.75, new Rotation2d(Math.PI / 2)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    public static Trajectory getGroundIntakeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                new Pose2d(0.3, -0.2, new Rotation2d(0)),
                new Pose2d(0.7, -0.4, new Rotation2d(5 * Math.PI / 3)),
        };

        return createTrajectory(waypoints, isReversed);
    }

    public static Trajectory getBlankTrajectory(Translation2d pos){
        Pose2d[] waypoints = new Pose2d[]{
            new Pose2d(pos, new Rotation2d(0)),
            new Pose2d(0.3,-0.2, new Rotation2d(0))
        };

        return createTrajectory(waypoints, false);

    }

    /***
     * 
     * @param desiredSetpoint ArmSetpoints: where you want to go
     * @param arm Arm: the arm subsystem object, used for its position and getting/setting current setpoint
     * @return Trajectory: the optimal trajectory for where you are to where you want to go
     */
    public static Trajectory findTrajectory(ArmSetpoints desiredSetpoint, Arm arm){
        ArmSetpoints currentSetpoint = arm.getCurrentSetpoint();
        Trajectory trajectory;

        if (currentSetpoint == ArmSetpoints.DOUBLE_SUBSTATION && desiredSetpoint != ArmSetpoints.STOWED) {
            System.out.println("Illigal double sub");
            arm.setCurrentSetpoint(ArmSetpoints.STOWED);
            return ArmTrajectories.getDoubleSubstationTrajectory(true);
        }
        if (currentSetpoint == ArmSetpoints.TOP_INTAKE && desiredSetpoint != ArmSetpoints.STOWED) {
            System.out.println("Illigal top intake");
            arm.setCurrentSetpoint(ArmSetpoints.STOWED);

            return ArmTrajectories.getGroundIntakeTrajectory(true);
        }

        if (desiredSetpoint == ArmSetpoints.CONE_TOP) {

            // if already there, reverse to stow
            if (desiredSetpoint == currentSetpoint) {
                System.out.println("reverse top node");

                arm.setCurrentSetpoint (ArmSetpoints.STOWED);
                trajectory = ArmTrajectories.getTopNodeTrajectory(true)
                        .concatenate(ArmTrajectories.getStingTrajectory(true));
                return trajectory;
            }
            // if at mid, run the node to node
            if (currentSetpoint == ArmSetpoints.CONE_MID) {
                System.out.println("mid to top node");

                trajectory = ArmTrajectories.getMidNodeToTopNodTrajectory(false);
            }
            // otherwise do it normally
            else {
                trajectory = ArmTrajectories.getTopNodeTrajectory(false);
                System.out.println("from sting to top node");

                // if at sting, run the trajectory. If not at sting, go to sting then run the
                // trajectory
                if (currentSetpoint != ArmSetpoints.STING) {

                    System.out.println("to sting to top node");
                    trajectory = ArmTrajectories.getStingTrajectory(false).concatenate(trajectory);
                }
            }
            // change the current setpoint and return
            arm.setCurrentSetpoint (ArmSetpoints.CONE_TOP);
            return trajectory;
        }

        else if (desiredSetpoint == ArmSetpoints.CONE_MID) {
            if (desiredSetpoint == currentSetpoint) {
                arm.setCurrentSetpoint (ArmSetpoints.STOWED);
                trajectory = ArmTrajectories.getMidNodeTrajectory(true)
                        .concatenate(ArmTrajectories.getStingTrajectory(true));
                return trajectory;
            }
            if (currentSetpoint == ArmSetpoints.CONE_TOP) {
                trajectory = ArmTrajectories.getMidNodeToTopNodTrajectory(true);
            } else {
                trajectory = ArmTrajectories.getMidNodeTrajectory(false);
                if (currentSetpoint != ArmSetpoints.STING) {
                    trajectory = ArmTrajectories.getStingTrajectory(false).concatenate(trajectory);
                }
            }
            arm.setCurrentSetpoint (ArmSetpoints.CONE_MID);
            return trajectory;
        }

        else if (desiredSetpoint == ArmSetpoints.DOUBLE_SUBSTATION) {
            if (desiredSetpoint == currentSetpoint) {
                arm.setCurrentSetpoint (ArmSetpoints.STOWED);

                trajectory = ArmTrajectories.getDoubleSubstationTrajectory(true);
                return trajectory;
            }
            trajectory = ArmTrajectories.getDoubleSubstationTrajectory(false);
            arm.setCurrentSetpoint (ArmSetpoints.DOUBLE_SUBSTATION);
            return trajectory;

        }

        else if (desiredSetpoint == ArmSetpoints.TOP_INTAKE) {
            if (desiredSetpoint == currentSetpoint) {
                arm.setCurrentSetpoint (ArmSetpoints.STOWED);

                trajectory = ArmTrajectories.getGroundIntakeTrajectory(true);
                return trajectory;
            }
            trajectory = ArmTrajectories.getGroundIntakeTrajectory(false);
            arm.setCurrentSetpoint (ArmSetpoints.TOP_INTAKE);
            return trajectory;
        }

        else if (desiredSetpoint == ArmSetpoints.STING) {
            if (desiredSetpoint == currentSetpoint) {
                arm.setCurrentSetpoint (ArmSetpoints.STOWED);

                trajectory = ArmTrajectories.getStingTrajectory(true);
                return trajectory;
            }
            trajectory = ArmTrajectories.getStingTrajectory(false);
            arm.setCurrentSetpoint (ArmSetpoints.STING);
            return trajectory;
        } else {
            return null;
        }    
    }

}
