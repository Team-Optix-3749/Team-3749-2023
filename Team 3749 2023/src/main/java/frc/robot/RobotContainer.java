package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.utils.Constants.AutoConstants.TopBottom;
import frc.robot.utils.Constants.VisionConstants.Pipelines;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);

    private final Swerve swerve = new Swerve();
    private final ArmIntake armIntake = new ArmIntake();
    private final Arm arm = new Arm();
    private final LEDs leds = new LEDs();
    private final Limelight limelight = new Limelight();

    private final JoystickIO joystickIO = new JoystickIO(pilot, operator, swerve, limelight, leds, armIntake,
            arm);

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

        RobotController.setBrownoutVoltage(6.75);
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

        if (!JoystickIO.didJoysticksChange())
            return;
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        joystickIO.getButtonBindings();

    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        return AutoCommands.getTopTwoPiece(swerve, arm, armIntake, limelight, leds);

    }

    /**
     * Set event maps for autonomous
     */
    public void configureAuto() {
        Constants.AutoConstants.eventMap.put("Pickup Cube",
                new SequentialCommandGroup(
                        Commands.runOnce(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage)),
                        new MoveArm(arm, armIntake, leds, ArmSetpoints.GROUND_INTAKE_CUBE)));
        Constants.AutoConstants.eventMap.put("Pickup Cone",
                new SequentialCommandGroup(
                        Commands.runOnce(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage)),
                        new MoveArm(arm, armIntake, leds, ArmSetpoints.GROUND_INTAKE_CONE)));
        Constants.AutoConstants.eventMap.put("Sting", new MoveArm(arm, armIntake, leds,
                ArmSetpoints.STING));
        Constants.AutoConstants.eventMap.put("Stow",
                new SequentialCommandGroup(
                        Commands.runOnce(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage)),
                        new MoveArm(arm, armIntake, leds, ArmSetpoints.STOW)));
        Constants.AutoConstants.eventMap.put("Place Mid", new MoveArm(arm, armIntake, leds,
                ArmSetpoints.PLACE_MID));
        Constants.AutoConstants.eventMap.put("Place Top", new MoveArm(arm, armIntake, leds,
                ArmSetpoints.PLACE_TOP));
        Constants.AutoConstants.eventMap.put("Wait", new WaitCommand(5));

    }
}