package frc.robot.commands.swerve;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/***
 * @author Noah Simon
 *         A class to return command sequences for auto
 *         Path Planner is used to create sequences, see docs below
 *         {@link} https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
 * 
 */

public final class AutoCommands {

    /***
     * 
     * @param swerveSubsystem the subsystem object. Do not make a new instance
     * @param trajectory      a viable trajectory object containing information
     *                        about where the robot should go
     * @param isFirstPath     if it is, it will reset odometry at its current
     *                        position
     * @return a SwerveControllerCommand based on the trajectory
     * @summary takes a trajectory and moves on it
     */

    public static Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath,
            Swerve swerveSubsystem) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        swerveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        swerveSubsystem::getPose, // Pose supplier
                        Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                        new PIDController(1.1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0
                                                      // will only use feedforwards.
                        new PIDController(1.1, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(2, 0, 0), // Rotation controller. Tune these values for your robot. Leaving
                                                    // them 0 will only use feedforwards.
                        swerveSubsystem::setModuleStates, // Module states consumer
                        false, // Should the path be automatically mirrored depending on alliance color.
                               // Optional, defaults to true
                        swerveSubsystem // Requires this drive subsystem
                ));
    }

    // Use parrelel command groups to run subsystems in parallel

    /***
     * 
     * @param swerveSubsystem the subsystem object. Do not make a new instance
     * @param trajectory      a viable trajectory object containing information
     *                        about where the robot should go
     * @return a SwerveControllerCommand based on the trajectory
     * @summary takes a trajectory and moves on it. Non path planner version
     */
    private static Command createCommandFromSwerveTrajectory(Swerve swerveSubsystem,
            Trajectory trajectory) {
        PIDController xController = new PIDController(0.1, 0, 0);
        PIDController yController = new PIDController(0.1, 0, 0);

        ProfiledPIDController thetaController = new ProfiledPIDController(0.5, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.DriveConstants.kAutoDriveMaxAngularSpeedRadiansPerSecond,
                        Constants.DriveConstants.kAutoDriveMaxAngularAccelerationUnitsPerSecond));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
                swerveSubsystem::getPose,
                Constants.DriveConstants.kDriveKinematics, xController, yController, thetaController,
                swerveSubsystem::setModuleStates, swerveSubsystem);
        return swerveControllerCommand;
    }

    /***
     * @summary takes in several translation 2D's and a relative end rotation.
     *          Creates a trajectory object. For non path planner use
     * @param waypoints   an array of translation 2D's. Each coordinate will be hit
     *                    in the order they are listed. The first should be 0,0
     * @param endRotation the relative-to-start rotation of the robot by the end of
     *                    the trajectory
     * @return a trajectory with the specified coordinates and end rotation
     */
    public static Trajectory createTrajectory(Translation2d[] waypoints, double endRotation) {
        // start and end coordinates, taken from waypoints
        double[] startXY = new double[] { waypoints[0].getX(), waypoints[0].getY() };
        double[] endXY = new double[] { waypoints[waypoints.length - 1].getX(),
                waypoints[waypoints.length - 1].getY() };

        Translation2d[] midpoints = new Translation2d[waypoints.length - 2];
        for (int i = 1; i < waypoints.length - 1; i++) {
            midpoints[i - 1] = waypoints[i];
        }

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
                Constants.DriveConstants.kAutoDriveMaxAccelerationUnitsPerSecond);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(startXY[0], startXY[1], new Rotation2d(0)),
                List.of(midpoints),
                new Pose2d(endXY[0], endXY[1], Rotation2d.fromDegrees(endRotation)),
                trajectoryConfig);

        return trajectory;
    }

    // A test method to outline a 1 by 1 meter square while rotating in a circle
    public static Command getTestSwerveCommand(Swerve swerveSubsystem) {
        Trajectory trajectory = createTrajectory(
                new Translation2d[] {
                        new Translation2d(0, 0),
                        new Translation2d(Units.feetToMeters(5), 0),
                }, 0);

        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                createCommandFromSwerveTrajectory(swerveSubsystem, trajectory),
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    // Essentially the template of a getPath command we should be using.
    public static Command getTestPathPlanner(Swerve swerveSubsystem, Alliance teamColor) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Distance Test", new PathConstraints(2.5, 2.5));

        trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, teamColor);
        Command path = new FollowPathWithEvents(followTrajectoryCommand(trajectory, true, swerveSubsystem),
                trajectory.getMarkers(), Constants.AutoConstants.eventMap);
        return new SequentialCommandGroup(
                path, Commands.run(() -> swerveSubsystem.zeroHeading(), swerveSubsystem).withTimeout(0.4),
                new AutoBalancingPID(swerveSubsystem));
    }
}
