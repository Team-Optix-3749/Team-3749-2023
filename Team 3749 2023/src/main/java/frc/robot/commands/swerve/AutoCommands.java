package frc.robot.commands.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.MoveArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.ArmIntake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.AutoConstants.TopBottom;
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
                        swerveSubsystem.resetAutoOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        swerveSubsystem::getAutoPose, // Pose supplier
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

    // Essentially the template of a getPath command we should be using.
    public static Command getTestPathPlanner(Swerve swerveSubsystem, Alliance teamColor) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("2 Piece", new PathConstraints(2.5, 2.5));
        trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, teamColor);
        Command path = new FollowPathWithEvents(followTrajectoryCommand(trajectory, true, swerveSubsystem),
                trajectory.getMarkers(), Constants.AutoConstants.eventMap);
        return new SequentialCommandGroup(
                path);
    }

    public static Command getMarkerTester(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake,
            Alliance teamColor, Constants.AutoConstants.TopBottom topBottom) {
        PathPlannerTrajectory first = PathPlanner.loadPath("Ground Pickup Test", new PathConstraints(0.75, 0.75));

        first = PathPlannerTrajectory.transformTrajectoryForAlliance(first,
                teamColor);

        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first,
                true, swerveSubsystem),
                first.getMarkers(), Constants.AutoConstants.eventMap);
        return new SequentialCommandGroup(
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.4),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage)).withTimeout(0.15),
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                new MoveArm(arm, armIntake, ArmSetpoints.STOW));

    }


    public static Command getFieldTest(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake, Alliance teamColor,
            TopBottom topBottom) {

        PathPlannerTrajectory first;
        first = PathPlanner.loadPath("Field Length", new PathConstraints(0.75, 0.75));

        first = PathPlannerTrajectory.transformTrajectoryForAlliance(first, teamColor);

        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first, true, swerveSubsystem),
                first.getMarkers(),
                Constants.AutoConstants.eventMap);
        return new SequentialCommandGroup(
                path_1);
    }
    public static Command getPickupTest(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake, Alliance teamColor,
            TopBottom topBottom) {

        PathPlannerTrajectory first;
        first = PathPlanner.loadPath("pickup", new PathConstraints(0.75, 0.75));

        first = PathPlannerTrajectory.transformTrajectoryForAlliance(first, teamColor);

        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first, true, swerveSubsystem),
                first.getMarkers(),
                Constants.AutoConstants.eventMap);
        return new SequentialCommandGroup(
                Commands.waitSeconds(0.1),
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP), Commands.waitSeconds(0.5),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage)).withTimeout(0.15),
                path_1);
    }

    public static Command getTwoPiece(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake,
            Alliance teamColor, TopBottom topBottom) {
        PathPlannerTrajectory first;
        if (topBottom == TopBottom.TOP) {
            first = PathPlanner.loadPath("TOP 2 Piece", new PathConstraints(2.5, 2.5));
        } else {
            first = PathPlanner.loadPath("BOTTOM 2 Piece", new PathConstraints(2.5, 2.5));
        }

        first = PathPlannerTrajectory.transformTrajectoryForAlliance(first, teamColor);

        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first, true, swerveSubsystem),
                first.getMarkers(), Constants.AutoConstants.eventMap);
        return new SequentialCommandGroup(
                Commands.waitSeconds(0.1),
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.5),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage)).withTimeout(0.15),
                path_1,
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.5),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseCubeVoltage)).withTimeout(0.15),
                new MoveArm(arm, armIntake, ArmSetpoints.STOW));
    }

    public static Command getThreePiece(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake,
            Alliance teamColor, Constants.AutoConstants.TopBottom topBottom) {
        PathPlannerTrajectory first;
        PathPlannerTrajectory second;
        if (topBottom == TopBottom.TOP) {
            first = PathPlanner.loadPath("TOP 2 Piece", new PathConstraints(2.5, 2.5));
            second = PathPlanner.loadPath("TOP 2 Piece 3 Piece",
                    new PathConstraints(2.5, 2.5));
        } else {
            first = PathPlanner.loadPath("BOTTOM 2 Piece", new PathConstraints(2.5, 2.5));
            second = PathPlanner.loadPath("BOTTOM 2 Piece 3 Piece",
                    new PathConstraints(2.5, 2.5));
        }
        first = PathPlannerTrajectory.transformTrajectoryForAlliance(first, teamColor);
        second = PathPlannerTrajectory.transformTrajectoryForAlliance(second, teamColor);

        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first, true, swerveSubsystem),
                first.getMarkers(), Constants.AutoConstants.eventMap);
        Command path_2 = new FollowPathWithEvents(followTrajectoryCommand(second, false, swerveSubsystem),
                second.getMarkers(), Constants.AutoConstants.eventMap);
        return new SequentialCommandGroup(
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.4),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage)).withTimeout(0.15),
                path_1,
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.4),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseCubeVoltage)).withTimeout(0.15),
                path_2,
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.4),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage)).withTimeout(0.15),
                new MoveArm(arm, armIntake, ArmSetpoints.STOW));

    }

    public static Command getTwoPieceCharge(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake,
            Alliance teamColor, Constants.AutoConstants.TopBottom topBottom) {
        PathPlannerTrajectory first;
        PathPlannerTrajectory second;
        if (topBottom == TopBottom.TOP) {
            first = PathPlanner.loadPath("TOP 2 Piece", new PathConstraints(2.5, 2.5));
            second = PathPlanner.loadPath("TOP 2 Piece Pickup Charge Station",
                    new PathConstraints(2.5, 2.5));
        } else {
            first = PathPlanner.loadPath("BOTTOM 2 Piece", new PathConstraints(2.5, 2.5));
            second = PathPlanner.loadPath("BOTTOM 2 Piece Pickup Charge Station",
                    new PathConstraints(2.5, 2.5));
        }

        first = PathPlannerTrajectory.transformTrajectoryForAlliance(first, teamColor);
        second = PathPlannerTrajectory.transformTrajectoryForAlliance(second, teamColor);

        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first, true, swerveSubsystem),
                first.getMarkers(), Constants.AutoConstants.eventMap);
        Command path_2 = new FollowPathWithEvents(followTrajectoryCommand(second, false, swerveSubsystem),
                second.getMarkers(), Constants.AutoConstants.eventMap);
        return new SequentialCommandGroup(
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.4),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage)).withTimeout(0.15),
                path_1,
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.4),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseCubeVoltage)).withTimeout(0.15),
                path_2);
    }

}
