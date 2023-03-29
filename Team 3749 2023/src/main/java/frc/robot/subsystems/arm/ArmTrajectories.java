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

/**
 * Generate arm trajectories
 * 
 * @author Noah Simon
 * @author Raadwan Masum
 **/
public class ArmTrajectories {

    /**
     * Create trajectory
     * 
     * @param waypoints
     * @return trajectory
     */
    private Trajectory createTrajectory(Pose2d[] waypoints, boolean isReversed) {
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
    private Trajectory makeStowToStingTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.STOW.toPose2d(Math.PI / 4),
                ArmSetpoints.STING.toPose2d(Math.PI / 2)
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from sting position and stow position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeCubeStowToStingTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.CUBE_STOW.toPose2d(Math.PI / 4),
                ArmSetpoints.STING.toPose2d(Math.PI / 2)
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from sting position and top node scoring position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeStingToTopNodeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.STING.toPose2d(Math.PI / 2.5),
                new Pose2d(new Translation2d(0.7, 1), new Rotation2d(Math.PI / 8)),
                ArmSetpoints.PLACE_TOP.toPose2d(0)
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from sting position and mid node scoring position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeStingToMidNodeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.STING.toPose2d(0),
                ArmSetpoints.PLACE_MID.toPose2d(Math.PI * 5 / 3)
        };

        return createTrajectory(waypoints, isReversed);
    }
    /**
     * Move arm to and from stow position and double substiation loading position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeStowToDoubleSubstationTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.STOW.toPose2d(Math.PI / 4),
                ArmSetpoints.DOUBLE_SUBSTATION.toPose2d(Math.PI / 2)
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from stow position and double substiation loading position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeCubeStowToDoubleSubstationTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.CUBE_STOW.toPose2d(Math.PI / 4),
                ArmSetpoints.DOUBLE_SUBSTATION.toPose2d(Math.PI / 2)
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from stow position and further ground intake position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeStowToCubeGroundIntakeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.STOW.toPose2d(0),
                ArmSetpoints.GROUND_INTAKE_CUBE.toPose2d(Math.PI * 5 / 3)
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from stow position and further ground intake position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeCubeStowToCubeGroundIntakeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.CUBE_STOW.toPose2d(0),
                ArmSetpoints.GROUND_INTAKE_CUBE.toPose2d(Math.PI * 5 / 3)
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from stow position and ground intake position, raised
     * slightly for cones
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeStowToConeGroundIntakeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.STOW.toPose2d(0),
                ArmSetpoints.GROUND_INTAKE_CONE.toPose2d(Math.PI * 5 / 3)
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm from mid node to top node position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeMidNodeToTopNodeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.PLACE_MID.toPose2d(Math.PI / 3),
                ArmSetpoints.PLACE_TOP.toPose2d(Math.PI / 8),

        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm from Stow to Cube Stow
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeStowToCubeStowTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.STOW.toPose2d(0),
                ArmSetpoints.CUBE_STOW.toPose2d(Math.PI/ 4 + Math.PI)

        };

        return createTrajectory(waypoints, isReversed);
    }

    // top
    private Trajectory stowToTop = makeStowToStingTrajectory(false).concatenate(makeStingToTopNodeTrajectory(false));
    private Trajectory cubeStowToTop = makeCubeStowToStingTrajectory(false)
            .concatenate(makeStingToTopNodeTrajectory(false));
    private Trajectory stingToTop = makeStingToTopNodeTrajectory(false);
    private Trajectory topToStow = makeStingToTopNodeTrajectory(true).concatenate(makeStowToStingTrajectory(true));
    private Trajectory topToCubeSstow = makeStingToTopNodeTrajectory(true)
            .concatenate(makeCubeStowToStingTrajectory(true));
    private Trajectory topToSting = makeStingToTopNodeTrajectory(true);
    private Trajectory topToMid = makeMidNodeToTopNodeTrajectory(true);

    // mid
    private Trajectory stowToMid = makeStowToStingTrajectory(false).concatenate(makeStingToMidNodeTrajectory(false));
    private Trajectory cubeStowToMid = makeCubeStowToStingTrajectory(false)
            .concatenate(makeStingToMidNodeTrajectory(false));
    private Trajectory stingToMid = makeStingToMidNodeTrajectory(false);
    private Trajectory midToStow = makeStingToMidNodeTrajectory(true).concatenate(makeStowToStingTrajectory(true));
    private Trajectory midToCubeStow = makeStingToMidNodeTrajectory(true)
            .concatenate(makeCubeStowToStingTrajectory(true));
    private Trajectory midToSting = makeStingToMidNodeTrajectory(true);
    private Trajectory midToTop = makeMidNodeToTopNodeTrajectory(false);

    // double sub
    private Trajectory stowToDoubleSub = makeStowToDoubleSubstationTrajectory(false);
    private Trajectory cubeStowToDoubleSub = makeCubeStowToDoubleSubstationTrajectory(false);
    private Trajectory doubleSubToStow = makeStowToDoubleSubstationTrajectory(true);
    private Trajectory doubleSubToCubeStow = makeCubeStowToDoubleSubstationTrajectory(true);

    // ground intake cube
    private Trajectory stowToGroundIntakeCube = makeStowToCubeGroundIntakeTrajectory(false);
    private Trajectory cubeStowToGroundIntakeCube = makeCubeStowToCubeGroundIntakeTrajectory(false);
    private Trajectory groundIntakeCubeToStow = makeStowToCubeGroundIntakeTrajectory(true);
    private Trajectory groundIntakeCubeToCubeStow = makeCubeStowToCubeGroundIntakeTrajectory(true);

    // ground intake cone
    private Trajectory stowToGroundIntakeCone = makeStowToConeGroundIntakeTrajectory(false);
    private Trajectory groundIntakeConeToStow = makeStowToConeGroundIntakeTrajectory(true);

    // Stow / Cube Stow / Sting
    private Trajectory stowToSting = makeStowToStingTrajectory(false);
    private Trajectory cubeStowToSting = makeCubeStowToStingTrajectory(false);
    private Trajectory stingToStow = makeStowToStingTrajectory(true);
    private Trajectory stingToCubeStow = makeCubeStowToStingTrajectory(true);
    private Trajectory stowToCubeStow = makeStowToCubeStowTrajectory(false);
    private Trajectory cubeStowToStow = makeStowToCubeStowTrajectory(true);
    public Trajectory getStowToTop() {
        return stowToTop;
    }

    public Trajectory getCubeStowToTop() {
        return cubeStowToTop;
    }

    public Trajectory getStingToTop() {
        return stingToTop;
    }

    public Trajectory getTopToStow() {
        return topToStow;
    }

    public Trajectory getTopToCubeStow() {
        return topToCubeSstow;
    }

    public Trajectory getTopToSting() {
        return topToSting;
    }

    public Trajectory getTopToMid() {
        return topToMid;
    }

    public Trajectory getStowToMid() {
        return stowToMid;
    }

    public Trajectory getCubeStowToMid() {
        return cubeStowToMid;
    }

    public Trajectory getStingToMid() {
        return stingToMid;
    }

    public Trajectory getMidToStow() {
        return midToStow;
    }

    public Trajectory getMidToCubeStow() {
        return midToCubeStow;
    }

    public Trajectory getMidToSting() {
        return midToSting;
    }

    public Trajectory getMidToTop() {
        return midToTop;
    }

    public Trajectory getStowToDoubleSub() {
        return stowToDoubleSub;
    }

    public Trajectory getCubeStowToDoubleSub() {
        return cubeStowToDoubleSub;
    }

    public Trajectory getDoubleSubToStow() {
        return doubleSubToStow;
    }

    public Trajectory getDoubleSubToCubeStow() {
        return doubleSubToCubeStow;
    }

    public Trajectory getStowToGroundIntakeCube() {
        return stowToGroundIntakeCube;
    }

    public Trajectory getCubeStowToGroundIntakeCube() {
        return cubeStowToGroundIntakeCube;
    }

    public Trajectory getGroundIntakeCubeToStow() {
        return groundIntakeCubeToStow;
    }

    public Trajectory getGroundIntakeCubeToCubeStow() {
        return groundIntakeCubeToCubeStow;
    }

    public Trajectory getStowToGroundIntakeCone() {
        return stowToGroundIntakeCone;
    }

    public Trajectory getGroundIntakeConeToStow() {
        return groundIntakeConeToStow;
    }

    public Trajectory getStowToSting() {
        return stowToSting;
    }

    public Trajectory getCubeStowToSting() {
        return cubeStowToSting;
    }

    public Trajectory getStingToStow() {
        return stingToStow;
    }

    public Trajectory getStingToCubeStow() {
        return stingToCubeStow;
    }

    public Trajectory getStowToCubeStow() {
        return stowToCubeStow;
    }

    public Trajectory getCubeStowToStow() {
        return cubeStowToStow;
    }


    
}