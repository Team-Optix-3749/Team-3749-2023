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
    private Trajectory makeStowToDoubleSubstationCubeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.STOW.toPose2d(Math.PI / 4),
                ArmSetpoints.DOUBLE_SUBSTATION_CUBE.toPose2d(Math.PI / 2)
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from stow position and double substiation loading position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeCubeStowToDoubleSubstationCubeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.CUBE_STOW.toPose2d(Math.PI / 4),
                ArmSetpoints.DOUBLE_SUBSTATION_CUBE.toPose2d(Math.PI / 2)
        };

        return createTrajectory(waypoints, isReversed);
    }
    /**
     * Move arm to and from stow position and double substiation loading position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeStowToDoubleSubstationConeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.STOW.toPose2d(Math.PI / 4),
                ArmSetpoints.DOUBLE_SUBSTATION_CONE.toPose2d(Math.PI / 2)
        };

        return createTrajectory(waypoints, isReversed);
    }

    /**
     * Move arm to and from stow position and double substiation loading position
     * 
     * @param isReversed
     * @return Trajectory
     */
    private Trajectory makeCubeStowToDoubleSubstationConeTrajectory(boolean isReversed) {
        Pose2d[] waypoints = new Pose2d[] {
                ArmSetpoints.CUBE_STOW.toPose2d(Math.PI / 4),
                ArmSetpoints.DOUBLE_SUBSTATION_CONE.toPose2d(Math.PI / 2)
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
            new Pose2d(0.7, 0.1, new Rotation2d(0)),
            ArmSetpoints.GROUND_INTAKE_CUBE.toPose2d(3 * Math.PI / 2)
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
            ArmSetpoints.STOW.toPose2d(0),
            new Pose2d(0.7, 0.1, new Rotation2d(0)),
            ArmSetpoints.GROUND_INTAKE_CUBE.toPose2d(3 * Math.PI / 2)
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
    private Trajectory stowToDoubleSubCone = makeStowToDoubleSubstationConeTrajectory(false);
    private Trajectory cubeStowToDoubleSubCone = makeCubeStowToDoubleSubstationConeTrajectory(false);
    private Trajectory doubleSubConeToStow = makeStowToDoubleSubstationConeTrajectory(true);
    private Trajectory doubleSubConeToCubeStow = makeCubeStowToDoubleSubstationConeTrajectory(true);

    private Trajectory stowToDoubleSubCube = makeStowToDoubleSubstationCubeTrajectory(false);
    private Trajectory cubeStowToDoubleSubCube = makeCubeStowToDoubleSubstationCubeTrajectory(false);
    private Trajectory doubleSubCubeToStow = makeStowToDoubleSubstationCubeTrajectory(true);
    private Trajectory doubleSubCubeToCubeStow = makeCubeStowToDoubleSubstationCubeTrajectory(true);

    // ground intake cube
    private Trajectory stowToGroundIntakeCube = makeStowToCubeGroundIntakeTrajectory(false);
    private Trajectory cubeStowToGroundIntakeCube = makeCubeStowToCubeGroundIntakeTrajectory(false);
    private Trajectory groundIntakeCubeToStow = makeStowToCubeGroundIntakeTrajectory(true);
    private Trajectory groundIntakeCubeToCubeStow = makeCubeStowToCubeGroundIntakeTrajectory(true);

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

    public Trajectory getStowToDoubleSubCone() {
        return stowToDoubleSubCone;
    }

    public Trajectory getCubeStowToDoubleSubCone() {
        return cubeStowToDoubleSubCone;
    }

    public Trajectory getDoubleSubConeToStow() {
        return doubleSubConeToStow;
    }

    public Trajectory getDoubleSubConeToCubeStow() {
        return doubleSubConeToCubeStow;
    }

    public Trajectory getStowToDoubleSubCube() {
        return stowToDoubleSubCube;
    }

    public Trajectory getCubeStowToDoubleSubCube() {
        return cubeStowToDoubleSubCube;
    }

    public Trajectory getDoubleSubCubeToStow() {
        return doubleSubCubeToStow;
    }

    public Trajectory getDoubleSubCubeToCubeStow() {
        return doubleSubCubeToCubeStow;
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