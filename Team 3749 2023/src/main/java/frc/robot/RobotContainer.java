package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.intake.*;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.swerve.AutoCommands;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.VisionConstants.Pipelines;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);

    private final Swerve swerve = new Swerve();
    private final ArmIntake armIntake = new ArmIntake();
    private final Arm arm = new Arm();
    private final ArmTrajectories armTrajectories = new ArmTrajectories();
    private final LEDs leds = new LEDs();
    private final Limelight limelight = new Limelight();

    private final JoystickIO joystickIO = new JoystickIO(pilot, operator, swerve, limelight, leds, armIntake,
            arm, armTrajectories);

    Command bottomTwoPiece = AutoCommands.getBottomTwoPiece(swerve, arm, armTrajectories, armIntake, limelight, leds);
    Command topTwoPiece = AutoCommands.getTopTwoPiece(swerve, arm, armTrajectories, armIntake, limelight, leds);
    Command bottomTwoPieceCharge = AutoCommands.getTopTwoPieceCharge(swerve, arm, armTrajectories, armIntake, limelight,
            leds);
    Command topTwoPieceCharge = AutoCommands.getTopTwoPieceCharge(swerve, arm, armTrajectories, armIntake, limelight,
            leds);
    Command middleCharge = AutoCommands.getMiddleCharge(swerve, arm, armTrajectories, armIntake, limelight, leds);
    Command apriltagAlign = AutoCommands.getAprilTagAlign(swerve, arm, armTrajectories, armIntake, limelight, leds);
    Command autoBalance = AutoCommands.getAutoBalanceTest(swerve, arm, armTrajectories, armIntake, limelight, leds);

    SendableChooser<Command> autoChooser = new SendableChooser<>();

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
            e.printStackTrace();
        }

        RobotController.setBrownoutVoltage(7.0);

        autoChooser.setDefaultOption("Top two piece charge", topTwoPieceCharge);
        autoChooser.addOption("Top Two PIece", topTwoPiece);
        autoChooser.addOption("Middle Charge", middleCharge);
        autoChooser.addOption("Bottom Two Piece", bottomTwoPiece);
        autoChooser.addOption("Bottom Two Piece Charge", bottomTwoPieceCharge);
        autoChooser.addOption("Apriltag Align", apriltagAlign);
        autoChooser.addOption("Auto Balance", autoBalance);

        SmartDashboard.putData(autoChooser);
    }

    /**
     * Set default commands
     */
    public void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveTeleopCommand(
                swerve,
                () -> -pilot.getLeftY(),
                () -> pilot.getLeftX(),
                () -> pilot.getRightX()));

        limelight.setDefaultCommand(
                Commands.run(
                        () -> {
                            limelight.setPipeline(Pipelines.APRILTAG.index);
                            limelight.updatePoseAprilTags(swerve.getPoseEstimator());
                        }, limelight));
    }

    /**
     * Set controller button bindings
     */
    public void configureButtonBindings() {

        // if (!JoystickIO.didJoysticksChange())
        // return;
        // CommandScheduler.getInstance().getActiveButtonLoop().clear();

        joystickIO.pilotAndOperatorBindings();

    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        return AutoCommands.getTopTwoPieceCharge(swerve, arm, armTrajectories, armIntake, limelight, leds);
        // return this.autoChooser.getSelected();
    }

    /**
     * Set event maps for autonomous
     */
    public void configureAuto() {
        Constants.AutoConstants.eventMap.put("Pickup Cube",
                new SequentialCommandGroup(
                        Commands.runOnce(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage)),
                        new MoveArm(arm, armTrajectories, armIntake, leds, ArmSetpoints.GROUND_INTAKE_CUBE)));
        Constants.AutoConstants.eventMap.put("Sting", new MoveArm(arm, armTrajectories, armIntake, leds,
                ArmSetpoints.STING));
        Constants.AutoConstants.eventMap.put("Stow",
                new MoveArm(arm, armTrajectories, armIntake, leds, ArmSetpoints.STOW));
        Constants.AutoConstants.eventMap.put("Cube Stow",
                new MoveArm(arm, armTrajectories, armIntake, leds, ArmSetpoints.CUBE_STOW));
        Constants.AutoConstants.eventMap.put("Place Mid", new MoveArm(arm, armTrajectories, armIntake, leds,
                ArmSetpoints.PLACE_MID));
        Constants.AutoConstants.eventMap.put("Place Top", new MoveArm(arm, armTrajectories, armIntake, leds,
                ArmSetpoints.PLACE_TOP));
        Constants.AutoConstants.eventMap.put("Wait", new WaitCommand(5));

    }
}