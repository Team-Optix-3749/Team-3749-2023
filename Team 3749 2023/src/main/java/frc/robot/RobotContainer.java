package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

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

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        DriverStation.removeRefreshedDataEventHandle(44000);

        configureButtonBindings();
        configureAuto();
        
        AutoConstants.autoChooser.addOption("Top Two Piece", AutoCommands.getTopTwoPiece(swerve, arm, armTrajectories, armIntake, limelight, leds));
        AutoConstants.autoChooser.addOption("Top Two Piece Charge", AutoCommands.getTopTwoPieceCharge(swerve, arm, armTrajectories, armIntake, limelight, leds));
        AutoConstants.autoChooser.addOption("Bottom Two Piece", AutoCommands.getBottomTwoPiece(swerve, arm, armTrajectories, armIntake, limelight, leds));
        AutoConstants.autoChooser.addOption("Bottom Two Piece Charge", AutoCommands.getBottomTwoPieceCharge(swerve, arm, armTrajectories, armIntake, limelight, leds));
        AutoConstants.autoChooser.addOption("Bottom Two Piece", AutoCommands.getBottomTwoPiece(swerve, arm, armTrajectories, armIntake, limelight, leds));
        AutoConstants.autoChooser.addOption("Middle Charge", AutoCommands.getMiddleCharge(swerve, arm, armTrajectories, armIntake, limelight, leds));
        AutoConstants.autoChooser.addOption("One Piece", AutoCommands.getOnePiece(swerve, arm, armTrajectories, armIntake, limelight, leds));
        // AutoConstants.autoChooser.addOption("PRINT", new PrintCommand("ALIHFALSDGFHASGDJFH"));

        AutoConstants.autoChooser.setDefaultOption("One Piece", AutoCommands.getOnePiece(swerve, arm, armTrajectories, armIntake, limelight, leds)   );

        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);

        try {
            FileWriter writer = new FileWriter("data.csv", false);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        RobotController.setBrownoutVoltage(7.0);

        ShuffleData.put("Swerve", AutoConstants.autoChooser);

    }

    /**
     * Set controller button bindings
     */
    public void configureButtonBindings() {

        // if (!JoystickIO.didJoysticksChange())
        // return;
        // CommandScheduler.getInstance().getActiveButtonLoop().clear();

        joystickIO.pilotAndOperatorBindings();

        joystickIO.setDefaultCommands();

    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        return AutoCommands.getBottomTwoPieceCharge(swerve, arm, armTrajectories, armIntake, limelight, leds);
        // return AutoCommands.getPieceAlign(swerve, arm, armTrajectories, armIntake, limelight, leds);
        // return AutoConstants.autoChooser.getSelected();
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
        Constants.AutoConstants.eventMap.put("Release Object",
                new SequentialCommandGroup(
                        Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage), armIntake)
                                .withTimeout(2),
                        Commands.runOnce(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage), armIntake)));

    }
}