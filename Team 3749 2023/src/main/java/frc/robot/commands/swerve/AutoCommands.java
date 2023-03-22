package frc.robot.commands.swerve;

import java.sql.Driver;
import java.util.spi.ToolProvider;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.arm.MoveArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.ArmIntake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.AutoConstants.TopBottom;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
    public static Command getWait(){
        System.out.println(DriverStation.getAlliance());
        return new WaitCommand(1);
    }
    
    public static Command getTest(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake, Limelight limelight,
            Constants.AutoConstants.TopBottom topBottom) {
        Alliance teamColor = DriverStation.getAlliance();

        PathPlannerTrajectory first = PathPlanner.loadPath("pickup", new PathConstraints(1, 1));
        first = PathPlannerTrajectory.transformTrajectoryForAlliance(first,
                teamColor);

        
        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first,
                true,  swerveSubsystem),
                first.getMarkers(), Constants.AutoConstants.eventMap);


        return new SequentialCommandGroup(

                path_1);
        


    }


    public static Command getAlexHouse(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake, Limelight limelight,
            Constants.AutoConstants.TopBottom topBottom) {
        Alliance teamColor = DriverStation.getAlliance();

        PathPlannerTrajectory first = PathPlanner.loadPath("Alex House", new PathConstraints(1, 1));
        first = PathPlannerTrajectory.transformTrajectoryForAlliance(first,
                teamColor);
        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first,
                true, swerveSubsystem),
                first.getMarkers(), Constants.AutoConstants.eventMap);

        PathPlannerTrajectory second = PathPlanner.loadPath("Alex House 3 Piece", new PathConstraints(1, 1));
        second = PathPlannerTrajectory.transformTrajectoryForAlliance(second,
                teamColor);
        Command path_2 = new FollowPathWithEvents(followTrajectoryCommand(second,
                false, swerveSubsystem),
                second.getMarkers(), Constants.AutoConstants.eventMap);

        return new SequentialCommandGroup(
                Commands.waitSeconds(0.1),
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.4),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                        .withTimeout(0.15),
                path_1,
                new ParallelDeadlineGroup(new SequentialCommandGroup(
                        Commands.waitSeconds(1),
                        Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                                .withTimeout(0.15)),
                        new ApriltagAlign(swerveSubsystem, limelight)),
                path_2,
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                        .withTimeout(0.15),
                // new ParallelDeadlineGroup(new SequentialCommandGroup(
                // Commands.waitSeconds(0.3),
                // Commands.run(() ->
                // armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                // .withTimeout(0.15)),
                // new RetroAlign(swerveSubsystem, limelight)),
                new MoveArm(arm, armIntake, ArmSetpoints.STOW),
                Commands.runOnce(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage), armIntake));

    }

    public static Command getTwoPiece(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake, Limelight limelight,
            TopBottom topBottom) {
        Alliance teamColor = DriverStation.getAlliance();

        PathPlannerTrajectory first;
        PathPlannerTrajectory second;
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
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage)).withTimeout(0.1),
                path_1,

                new ParallelDeadlineGroup(new SequentialCommandGroup(
                        Commands.waitSeconds(0.5),
                        Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                                .withTimeout(0.1)),
                        new ApriltagAlign(swerveSubsystem, limelight)),

                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage)).withTimeout(0.15),
                new MoveArm(arm, armIntake, ArmSetpoints.STOW));
    }

    public static Command getThreePiece(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake, Limelight limelight,
            Constants.AutoConstants.TopBottom topBottom) {
        Alliance teamColor = DriverStation.getAlliance();

        PathPlannerTrajectory first;
        PathPlannerTrajectory second;
        if (topBottom == TopBottom.TOP) {
            first = PathPlanner.loadPath("TOP 2 Piece", new PathConstraints(2.5, 2.5));
            second = PathPlanner.loadPath("TOP 2 Piece 3 Piece",
                    new PathConstraints(2.0, 2.0));
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
                Commands.waitSeconds(0.1),
                new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                Commands.waitSeconds(0.5),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage)).withTimeout(0.1),
                path_1,

                new ParallelDeadlineGroup(new SequentialCommandGroup(
                        Commands.waitSeconds(0.5),
                        Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                                .withTimeout(0.1)),
                        new ApriltagAlign(swerveSubsystem, limelight)),

                // path_2,
                // new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
                // Commands.waitSeconds(0.4),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage)).withTimeout(0.15),
                new MoveArm(arm, armIntake, ArmSetpoints.STOW));

    }

    public static Command getTwoPieceCharge(Swerve swerveSubsystem, Arm arm, ArmIntake armIntake, Limelight limelight,
            Constants.AutoConstants.TopBottom topBottom) {
        Alliance teamColor = DriverStation.getAlliance();

        PathPlannerTrajectory first;
        PathPlannerTrajectory second;
        if (topBottom == TopBottom.TOP) {
            first = PathPlanner.loadPath("TOP 2 Piece", new PathConstraints(2.5, 2.5));
            second = PathPlanner.loadPath("TOP 2 Piece - Charge",
                    new PathConstraints(2.5, 2.5));
        } else {
            first = PathPlanner.loadPath("BOTTOM 2 Piece", new PathConstraints(2.5, 2.5));
            second = PathPlanner.loadPath("BOTTOM 2 Piece - Charge",
                    new PathConstraints(2.5, 2.5));
        }

        first = PathPlannerTrajectory.transformTrajectoryForAlliance(first, teamColor);
        second = PathPlannerTrajectory.transformTrajectoryForAlliance(second, teamColor);

        Command path_1 = new FollowPathWithEvents(followTrajectoryCommand(first, true, swerveSubsystem),
                first.getMarkers(), Constants.AutoConstants.eventMap);
        Command path_2 = new FollowPathWithEvents(followTrajectoryCommand(second, false, swerveSubsystem),
                second.getMarkers(), Constants.AutoConstants.eventMap);
        return new SequentialCommandGroup(
            Commands.waitSeconds(0.1),
            new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP),
            Commands.waitSeconds(0.5),
            Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage)).withTimeout(0.1),
            path_1,
            
            new ParallelDeadlineGroup(new SequentialCommandGroup(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                            .withTimeout(0.1)),
                    new ApriltagAlign(swerveSubsystem, limelight)),
            path_2,
            new AutoBalancingPID(swerveSubsystem));
    }
}
