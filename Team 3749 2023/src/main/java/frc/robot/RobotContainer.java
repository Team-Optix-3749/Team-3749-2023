package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.intake.*;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.swerve.AlignHeading;
import frc.robot.commands.swerve.ApriltagAlign;
import frc.robot.commands.swerve.AutoCommands;
import frc.robot.commands.swerve.MoveToPose;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.vision.VisionDefaultCommand;
import frc.robot.commands.swerve.RetroAlign;
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);

    // Subsystems
    private final Swerve swerve = new Swerve();
    private final ArmIntake armIntake = new ArmIntake();
    private final SideIntake sideIntake = new SideIntake();
    private final Arm arm = new Arm();
    private final Limelight limelight = new Limelight();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        DriverStation.removeRefreshedDataEventHandle(44000);

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

        armIntake.setDefaultCommand(
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage), armIntake));

        sideIntake.setDefaultCommand(
                Commands.run(() -> sideIntake.setIntakeVoltage(Constants.SideIntake.idleVoltage), sideIntake));
        limelight.setDefaultCommand(new VisionDefaultCommand(limelight, swerve.getPoseEstimator()));

    }

    /**
     * Set controller button bindings
     * 
     */
    private void configureButtonBindings() {
        pilot.aWhileHeld(
                new SequentialCommandGroup(
                        new AlignHeading(swerve).withTimeout(1),
                        new RetroAlign(swerve, limelight)));

        pilot.bWhileHeld(
            new AlignHeading(swerve).withTimeout(1)
        );

        // arm setpoints (buttons)
        // pilot.a().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP));
        // pilot.b().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.PLACE_MID));
        pilot.x().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.GROUND_INTAKE));
        pilot.y().onTrue(Commands.runOnce(() -> sideIntake.toggleLiftSetpoint(), sideIntake));

        // arm setpoints (bumpers)
        pilot.rightBumper().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.STING));
        pilot.leftBumper().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.DOUBLE_SUBSTATION));

        // intake button bindings
        pilot.rightTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.releaseObjectVoltage));
        pilot.leftTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage));

        // swerve button bindings
        pilot.backWhileHeld(() -> swerve.zeroHeading(), swerve);

        // swerve rotation cardinals
        pilot.povUp().whileTrue(Commands.run(() -> swerve.turnToRotation(0)));
        pilot.povLeft().whileTrue(Commands.run(() -> swerve.turnToRotation(270)));
        pilot.povDown().whileTrue(Commands.run(() -> swerve.turnToRotation(180)));
        pilot.povRight().whileTrue(Commands.run(() -> swerve.turnToRotation(90)));

        pilot.startWhileHeld(
                Commands.run(() -> swerve.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));

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