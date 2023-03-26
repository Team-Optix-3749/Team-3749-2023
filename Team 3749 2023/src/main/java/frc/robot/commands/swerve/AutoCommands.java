package frc.robot.commands.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/***
 * @author Noah Simon
 *         A class to return command sequences for auto
 *         Path Planner is used to create sequences, see docs below
 *         {@link} https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
 */
public final class AutoCommands {
    /***
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
                        swerveSubsystem.resetGyro();
                        swerveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        () -> swerveSubsystem.getPose(), // Pose supplier
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

    public static Command getTopTaxi(Swerve swerveSubsystem) {
        PathPlannerTrajectory first = null;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            first = PathPlanner.loadPath("BLUE - TOP Taxi", new PathConstraints(1.5, 1.5));
        } else {
            first = PathPlanner.loadPath("RED - TOP Taxi", new PathConstraints(1.5, 1.5));

        }

        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first, true, swerveSubsystem),
                first.getMarkers(), Constants.AutoConstants.eventMap);

        return new SequentialCommandGroup(
                path_1);
    }

    public static Command getMiddleTaxi(Swerve swerveSubsystem) {
        PathPlannerTrajectory first = null;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            first = PathPlanner.loadPath("BLUE - MIDDLE Taxi", new PathConstraints(1.5, 1.5));
        } else {
            first = PathPlanner.loadPath("RED - MIDDLE Taxi", new PathConstraints(1.5, 1.5));

        }

        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first, true, swerveSubsystem),
                first.getMarkers(), Constants.AutoConstants.eventMap);

        return new SequentialCommandGroup(
                path_1);
    }

    public static Command getBottomTaxi(Swerve swerveSubsystem) {
        PathPlannerTrajectory first = null;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            first = PathPlanner.loadPath("BLUE - BOTTOM Taxi", new PathConstraints(1.5, 1.5));
        } else {
            first = PathPlanner.loadPath("RED - BOTTOM Taxi", new PathConstraints(1.5, 1.5));

        }

        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first, true, swerveSubsystem),
                first.getMarkers(), Constants.AutoConstants.eventMap);

        return new SequentialCommandGroup(
                path_1);
    }

}
