package frc.robot.commands.swerve;

import java.util.List;
import java.util.function.Consumer;
import frc.robot.Robot;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.vision.AlignApriltag;
import frc.robot.commands.vision.AlignPiece;
import frc.robot.subsystems.arm.ArmSparkMax;
import frc.robot.subsystems.arm.ArmTrajectories;
import frc.robot.subsystems.intake.ArmIntake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.VisionConstants.Piece;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/***
 * @author Noah Sin
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
                        swerve.setFlipGyro(true);
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

    public static Command getPlaceTop() {
        return new SequentialCommandGroup(
                Commands.waitSeconds(0.1),
                new MoveArm(ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.35),
                Commands.run(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                        .withTimeout(0.125),
                Commands.runOnce(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.idleVoltage)).withTimeout(0.1));
    }

    public static Command getBottomTwoPiece() {
        List<PathPlannerTrajectory> pathGroup = null;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            pathGroup = PathPlanner.loadPathGroup("BLUE - BOTTOM 2 Piece", new PathConstraints(3, 3));

        } else if (DriverStation.getAlliance() == Alliance.Red) {
            pathGroup = PathPlanner.loadPathGroup("RED - BOTTOM 2 Piece", new PathConstraints(3, 3));
        }
        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(0), true, Robot.swerve),
                pathGroup.get(0).getMarkers(), Constants.AutoConstants.eventMap);
        Command path_2 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(1), false, Robot.swerve),
                pathGroup.get(1).getMarkers(), Constants.AutoConstants.eventMap);
        Pose2d midPose = pathGroup.get(1).getInitialHolonomicPose();

        return new SequentialCommandGroup(
                new PrintCommand(DriverStation.getAlliance().toString() + " bottom two piece"),
                getPlaceTop(),
                path_1,
                new AlignPiece().withTimeout(1.75),
                Commands.runOnce(() -> Robot.swerve.resetOdometry(midPose)),
                path_2,
                new AlignApriltag().withTimeout(0.9),
                Commands.run(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                        .withTimeout(0.12),
                Commands.runOnce(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.idleVoltage)),
                new MoveArm(ArmSetpoints.STOW));

    }

    public static Command getTopTwoPiece() {
        List<PathPlannerTrajectory> pathGroup = null;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            pathGroup = PathPlanner.loadPathGroup("BLUE - TOP 2 Piece", new PathConstraints(2.5, 2.5));

        } else if (DriverStation.getAlliance() == Alliance.Red) {
            pathGroup = PathPlanner.loadPathGroup("RED - TOP 2 Piece", new PathConstraints(2.5, 2.5));
        }
        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(0), true, Robot.swerve),
                pathGroup.get(0).getMarkers(), Constants.AutoConstants.eventMap);
        Command path_2 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(1), false, Robot.swerve),
                pathGroup.get(1).getMarkers(), Constants.AutoConstants.eventMap);
        Pose2d midPose = pathGroup.get(1).getInitialHolonomicPose();

        return new SequentialCommandGroup(
                getPlaceTop(),
                path_1,
                // new AlignPiece().withTimeout(2),
                Commands.runOnce(() -> Robot.swerve.resetOdometry(midPose)),
                path_2,
                // new AlignApriltag().withTimeout(1.2),
                Commands.run(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                        .withTimeout(0.12),
                Commands.runOnce(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.idleVoltage)),
                new MoveArm(ArmSetpoints.STOW));
    }

    public static Command getTopTwoPieceCharge() {
        List<PathPlannerTrajectory> pathGroup = null;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            pathGroup = PathPlanner.loadPathGroup("BLUE - TOP 2 Piece", new PathConstraints(2.5, 2.5));

        } else if (DriverStation.getAlliance() == Alliance.Red) {
            pathGroup = PathPlanner.loadPathGroup("RED - TOP 2 Piece", new PathConstraints(2, 2));
        }
        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(0), true, Robot.swerve),
                pathGroup.get(0).getMarkers(), Constants.AutoConstants.eventMap);
        Command path_2 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(1), false, Robot.swerve),
                pathGroup.get(1).getMarkers(), Constants.AutoConstants.eventMap);
        Command path_3 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(2), false, Robot.swerve),
                pathGroup.get(2).getMarkers(), Constants.AutoConstants.eventMap);
        double goalHeading = DriverStation.getAlliance() == Alliance.Blue ? 180 : 0;

        Pose2d midPose = pathGroup.get(1).getInitialHolonomicPose();

        return new SequentialCommandGroup(
                getPlaceTop(),
                path_1,
                new AlignPiece().withTimeout(1.25),
                Commands.runOnce(() -> Robot.swerve.resetOdometry(midPose)),
                path_2,
                new AlignApriltag().withTimeout(1.2),
                Commands.run(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                        .withTimeout(0.12),
                Commands.runOnce(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.idleVoltage)),
                path_3,
                new AutoBalancingPID(Robot.swerve, goalHeading));
    }

    public static Command getBottomTwoPieceCharge() {
        List<PathPlannerTrajectory> pathGroup = null;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            pathGroup = PathPlanner.loadPathGroup("BLUE - BOTTOM 2 Piece", new PathConstraints(3, 3));

        } else if (DriverStation.getAlliance() == Alliance.Red) {
            pathGroup = PathPlanner.loadPathGroup("RED - BOTTOM 2 Piece", new PathConstraints(3, 3));
        }
        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(0), true, Robot.swerve),
                pathGroup.get(0).getMarkers(), Constants.AutoConstants.eventMap);
        Command path_2 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(1), false, Robot.swerve),
                pathGroup.get(1).getMarkers(), Constants.AutoConstants.eventMap);

        Command path_3 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(2), false, Robot.swerve),
                pathGroup.get(2).getMarkers(), Constants.AutoConstants.eventMap);

        Pose2d midPose = pathGroup.get(1).getInitialHolonomicPose();

        double goalHeading = DriverStation.getAlliance() == Alliance.Blue ? 0 : 180;
        return new SequentialCommandGroup(
                getPlaceTop(),
                path_1,
                new AlignPiece().withTimeout(2),
                Commands.runOnce(() -> Robot.swerve.resetOdometry(midPose)),
                path_2,
                new AlignApriltag().withTimeout(0.95),
                Commands.run(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                        .withTimeout(0.12),
                Commands.runOnce(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.idleVoltage)),
                path_3,
                new AutoBalancingPID(Robot.swerve, goalHeading));
    }

    public static Command getTopThreePiece() {
        List<PathPlannerTrajectory> pathGroup = null;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            pathGroup = PathPlanner.loadPathGroup("BLUE - TOP 3 Piece", new PathConstraints(3, 3));

        } else if (DriverStation.getAlliance() == Alliance.Red) {
            pathGroup = PathPlanner.loadPathGroup("RED - TOP 3 Piece", new PathConstraints(3, 3));
        }
        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(0), true, Robot.swerve),
                pathGroup.get(0).getMarkers(), Constants.AutoConstants.eventMap);
        Command path_2 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(1), false, Robot.swerve),
                pathGroup.get(1).getMarkers(), Constants.AutoConstants.eventMap);
        Command path_3 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(2), false, Robot.swerve),
                pathGroup.get(2).getMarkers(), Constants.AutoConstants.eventMap);

        Pose2d midPose1 = pathGroup.get(1).getInitialHolonomicPose();
        Pose2d midPose2 = pathGroup.get(2).getInitialHolonomicPose();

        return new SequentialCommandGroup(
                getPlaceTop(),
                path_1,
                new AlignPiece().withTimeout(1.25),
                Commands.runOnce(() -> Robot.swerve.resetOdometry(midPose1)),
                path_2,
                new AlignPiece().withTimeout(1.25),
                Commands.runOnce(() -> Robot.swerve.resetOdometry(midPose2)),
                path_3,

                Commands.run(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                        .withTimeout(0.12),
                Commands.runOnce(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.idleVoltage)),
                new MoveArm(ArmSetpoints.STOW, false));
    }

    public static Command getOnePiece() {

        return new SequentialCommandGroup(
                getPlaceTop(),
                Commands.runOnce(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.idleVoltage)).withTimeout(0.1),
                new MoveArm(ArmSetpoints.STOW));
    }

    public static Command getAprilTagAlign() {
        return new SequentialCommandGroup(
                getPlaceTop());
    }

    public static Command getTest() {
        PathPlannerTrajectory path = PathPlanner.loadPath("BLUE - TEST", new PathConstraints(1, 1));
        return new SequentialCommandGroup(new FollowPathWithEvents(followTrajectoryCommand(path, true, Robot.swerve), path.getMarkers(),
                Constants.AutoConstants.eventMap), Commands.runOnce(() -> Robot.swerve.stop()), new PrintCommand("AUTO DONE"));
    }

    public static Command getPieceAlign() {
        List<PathPlannerTrajectory> pathGroup = null;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            pathGroup = PathPlanner.loadPathGroup("BLUE - Align Piece Test", new PathConstraints(1, 1));

        } else if (DriverStation.getAlliance() == Alliance.Red) {
            pathGroup = PathPlanner.loadPathGroup("RED - Align Piece Test", new PathConstraints(1, 1));
        }
        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(0), true, Robot.swerve),
                pathGroup.get(0).getMarkers(), Constants.AutoConstants.eventMap);
        Command path_2 = new FollowPathWithEvents(followTrajectoryCommand(pathGroup.get(1), false, Robot.swerve),
                pathGroup.get(1).getMarkers(), Constants.AutoConstants.eventMap);

        Pose2d midPose = pathGroup.get(1).getInitialHolonomicPose();

        return new SequentialCommandGroup(
                Commands.runOnce(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.idleVoltage)).withTimeout(0.1),
                path_1,
                new AlignPiece().withTimeout(15),
                Commands.runOnce(() -> Robot.swerve.resetOdometry(midPose)),

                path_2);
    }

}