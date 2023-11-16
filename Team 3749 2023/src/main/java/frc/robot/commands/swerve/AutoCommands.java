package frc.robot.commands.swerve;

import java.util.function.Consumer;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;


/***
 * @author Noah Simon
 *         A class to return command sequences for auto
 *         Path Planner is used to create sequences, see docs below
 *         {@link} https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
 */
public final class AutoCommands {
    public static Consumer<Pose2d> pathTargetPose = pose -> SmartDashboard.putNumberArray("Auto Path Pose Targets",
            new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });

    /***
     * @param swerve      the subsystem object. Do not make a new instance
     * @param trajectory  a viable trajectory object containing information
     *                    about where the robot should go
     * @param isFirstPath if it is, it will reset odometry at its current
     *                    position
     * @return a SwerveControllerCommand based on the trajectory
     * @summary takes a trajectory and moves on it
     */
    private static Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath,
            Swerve swerve) {
        PPSwerveControllerCommand.setLoggingCallbacks(null, pathTargetPose, null, null);
    
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        swerve.resetGyro();
                        swerve.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        () -> swerve.getPose(), // Pose supplier
                        Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                        new PIDController(1.1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0
                                                      // will only use feedforwards.
                        new PIDController(1.1, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(2.2, 0, 0), // Rotation controller. Tune these values for your robot. Leaving
                                                      // them 0 will only use feedforwards.
                        swerve::setModuleStates, // Module states consumer
                        false, // Should the path be automatically mirrored depending on alliance color.
                               // Optional, defaults to true
                        swerve // Requires this drive subsystem
                ));
    }

    public static Command getTestPath(Swerve swerve) {
        PathPlannerTrajectory path;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            path = PathPlanner.loadPath("Blue - TEST", new PathConstraints(1.5, 1.5));
        } else {
            path = PathPlanner.loadPath("", new PathConstraints(1.5, 1.5));
        }

        Command pathCommand = new FollowPathWithEvents(followTrajectoryCommand(path, true, swerve), path.getMarkers(),
                Constants.AutoConstants.eventMap);
        return new SequentialCommandGroup(
                pathCommand);

    }

}