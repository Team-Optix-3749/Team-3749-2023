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


public final class AutoCommands {

    public static Command getTestSwerveCommand(SwerveSubsystem swerveSubsystem){
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
        Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 0),
            new Translation2d(1, -1)),
        new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        trajectoryConfig);

    PIDController xController = new PIDController(0.5, 0, 0);
    PIDController yController = new PIDController(0.5, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(0.5, 0, 0,
        new TrapezoidProfile.Constraints(Constants.DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond,
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose,
        Constants.DriveConstants.kDriveKinematics, xController, yController, thetaController,
        swerveSubsystem::setModuleStates, swerveSubsystem);

    return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
    

