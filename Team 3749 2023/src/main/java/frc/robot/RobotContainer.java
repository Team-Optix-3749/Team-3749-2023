package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.claw.*;
import frc.robot.commands.arm.ArmFollowTrajectory;
import frc.robot.commands.swerve.AutoCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.utils.*;
import frc.robot.utils.Constants;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);

    // Subsystems
    private final Swerve swerve = new Swerve();
    private final Claw claw = new Claw();
    private final Arm arm = new Arm();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        configureDefaultCommands();
        configureButtonBindings();
        configureAuto();

        try {
            FileWriter writer = new FileWriter("data.csv", false);
            writer.close();
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
    }

    /**
     * Set default commands
     * 
     */
    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveTeleopCommand(
                swerve,
                () -> -pilot.getLeftY(),
                () -> pilot.getLeftX(),
                () -> pilot.getRightX()));

        claw.setDefaultCommand(
                Commands.run(() -> claw.setVoltage(1.0), claw));
        // arm.setDefaultCommand(new SequentialCommandGroup(
        // new ArmFollowTrajectory(arm,
        // ArmTrajectories.getTopNodeTrajectoryPose(false)),
        // new ArmFollowTrajectory(arm,
        // ArmTrajectories.getTopNodeTrajectoryPose(true))));
    }

    /**
     * Set controller button bindings
     * 
     */
    private void configureButtonBindings() {
        pilot.a().whileTrue(
                new SequentialCommandGroup(
                        new ArmFollowTrajectory(arm, ArmTrajectories.getTopNodeTrajectory(false)),
                        new WaitCommand(1),
                        Commands.run(() -> claw.setVoltage(-1)).withTimeout(1),
                        new ArmFollowTrajectory(arm, ArmTrajectories.getTopNodeTrajectory(true))))
                .whileFalse(new PrintCommand("false"));

        pilot.b().whileTrue(
                new SequentialCommandGroup(
                        new ArmFollowTrajectory(arm, ArmTrajectories.getMidNodeTrajectory(false)),
                        Commands.run(() -> claw.setVoltage(-1)).withTimeout(1),
                        new ArmFollowTrajectory(arm, ArmTrajectories.getMidNodeTrajectory(true))))
                .whileFalse(new PrintCommand("false"));

        pilot.rightBumper().whileTrue(
                new SequentialCommandGroup(
                        new ArmFollowTrajectory(arm, ArmTrajectories.getStingTrajectory(false))))
                .whileFalse(new PrintCommand("false"));

        pilot.leftBumper().whileTrue(
                new SequentialCommandGroup(
                        new ArmFollowTrajectory(arm, ArmTrajectories.getStingTrajectory(true))))
                .whileFalse(new PrintCommand("false"));

        pilot.rightTrigger().whileTrue(
                new SequentialCommandGroup(
                        new ArmFollowTrajectory(arm, ArmTrajectories.getDoubleSubstationTrajectory(false))))
                .onFalse(new ArmFollowTrajectory(arm, ArmTrajectories.getDoubleSubstationTrajectory(true)));

        pilot.rightTrigger().whileTrue(Commands.run(() -> claw.setVoltage(6)));
        pilot.leftTrigger().whileTrue(Commands.run(() -> claw.setVoltage(-3)));

        pilot.x().whileTrue(Commands.run(() -> claw.setVoltage(3)));

        pilot.backWhileHeld(() -> swerve.zeroHeading(), swerve);
    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        return AutoCommands.getTestPathPlanner(swerve, Alliance.Blue);
    }

    /**
     * Set event maps for autonomous
     */
    public void configureAuto() {
        Constants.AutoConstants.eventMap.put("pickup_cone_floor", Commands.print("PICKUP CONE FLOOR"));
        Constants.AutoConstants.eventMap.put("pickup_cube_floor", null);
        Constants.AutoConstants.eventMap.put("pickup_cone_double_substation", null);
        Constants.AutoConstants.eventMap.put("pickup_cube_double_substation", null);
        Constants.AutoConstants.eventMap.put("pickup_cone_single_substation", null);
        Constants.AutoConstants.eventMap.put("pickup_cube_single_substation", null);
        Constants.AutoConstants.eventMap.put("place_cone_bottom", null);
        Constants.AutoConstants.eventMap.put("place_cube_bottom", null);
        Constants.AutoConstants.eventMap.put("place_cone_mid", null);
        Constants.AutoConstants.eventMap.put("place_cube_mid", null);
        Constants.AutoConstants.eventMap.put("place_cone_top", null);
        Constants.AutoConstants.eventMap.put("place_cube_top", null);
    }
}