package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/***
 * A class to return command sequences for auto
 * @author Noah Simon
 * 
 */

public final class AutoCommands {
    /***
     * 
     * @param swerveSubsystem the subsystem object. Do not make a new instance
     * @param trajectory a viable trajectory object containing information about where the robot should go
     * @return a SwerveControllerCommand based on the trajectory
     * @summary Edit this method for things like PID tuning and Max speed changes
     */
    private static Command createCommandFromSwerveTrajectory(SwerveSubsystem swerveSubsystem, Trajectory trajectory) {
        PIDController xController = new PIDController(0.5, 0, 0);
        PIDController yController = new PIDController(0.5, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(0.5, 0, 0,
                new TrapezoidProfile.Constraints(2 * Math.PI,
                        Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
                swerveSubsystem::getPose,
                Constants.DriveConstants.kDriveKinematics, xController, yController, thetaController,
                swerveSubsystem::setModuleStates, swerveSubsystem);
        return swerveControllerCommand;

    }

    /***
     * @summary takes in several translation 2D's and a relative end rotation.
     *          Creates a trajectory object
     * @param waypoints   an array of translation 2D's. Each coordinate will be hit
     *                    in the order they are listed. The first should be 0,0
     * @param endRotation the relative-to-start rotation of the robot by the end of
     *                    the trajectory
     * @return a trajectory with the specified coordinates and end rotation
     */
    private static Trajectory createTrajectory(Translation2d[] waypoints, double endRotation) {
        double[] startXY = new double[] { waypoints[0].getX(), waypoints[0].getY() };
        double[] endXY = new double[] { waypoints[waypoints.length - 1].getX(),
                waypoints[waypoints.length - 1].getY() };

        Translation2d[] midpoints = new Translation2d[waypoints.length - 2];
        for (int i = 1; i < waypoints.length; i++) {
            midpoints[i - 1] = waypoints[i];
        }

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
                Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(startXY[0], startXY[1], new Rotation2d(0)),
                List.of(midpoints),
                new Pose2d(endXY[0], endXY[1], Rotation2d.fromDegrees(endRotation)),
                trajectoryConfig);

        return trajectory;
    }

    // A test method to outline a 1 by 1 meter square while rotating in a circle
    public static Command getTestSwerveCommand(SwerveSubsystem swerveSubsystem) {
        Trajectory trajectory = createTrajectory(
                new Translation2d[] {
                        new Translation2d(0, 0),
                        new Translation2d(1, 0),
                        new Translation2d(1, 1),
                        new Translation2d(0, 1),
                        new Translation2d(0, 0)
                }, 360);

        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                createCommandFromSwerveTrajectory(swerveSubsystem, trajectory),
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
