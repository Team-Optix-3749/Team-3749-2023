package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.swerve.AutoBalancingPID;
import frc.robot.commands.swerve.AutoCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.vision.VisionDefaultCommand;
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.AutoConstants.TopBottom;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);

    // Subsystems
    private final Swerve swerve = new Swerve();
    private final ArmIntake armIntake = new ArmIntake();
    private final SideIntake sideIntake = new SideIntake();
    private final Arm arm = new Arm();
    private final Limelight limelight = new Limelight();

    JoystickIO joystickOI = new JoystickIO(pilot, operator, swerve, limelight, armIntake, sideIntake, arm);

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
    public void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveTeleopCommand(
                swerve,
                () -> -pilot.getLeftY(),
                () -> pilot.getLeftX(),
                () -> pilot.getRightX()));

        // armIntake.setDefaultCommand(
        // Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage),
        // armIntake));

        sideIntake.setDefaultCommand(
                Commands.run(() -> sideIntake.setIntakeVoltage(Constants.SideIntake.idleVoltage), sideIntake));

        limelight.setDefaultCommand(new VisionDefaultCommand(limelight, swerve.getPoseEstimator()));
    }

    /**
     * Set controller button bindings
     */
    public void configureButtonBindings() {

        if (!JoystickIO.didJoysticksChange())
            return;
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        joystickOI.getButtonBindings();

    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        // return AutoCommands.getTest(swerve, arm, armIntake, limelight, TopBottom.TOP);
        return AutoCommands.getTwoPiece(swerve, arm, armIntake, limelight);
        // return AutoCommands.getTwoPiece(swerve, arm, armIntake,
        // TopBottom.TOP);
        // return AutoCommands.getThreePiece(swerve, arm, armIntake,
        // TopBottom.TOP);

    }

    /**
     * Set event maps for autonomous
     */
    public void configureAuto() {
        Constants.AutoConstants.eventMap.put("Pickup Cube",
                new SequentialCommandGroup(
                        Commands.runOnce(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage)),
                        new MoveArm(arm, armIntake, ArmSetpoints.GROUND_INTAKE_CUBE)));
        Constants.AutoConstants.eventMap.put("Pickup Cone",
                new SequentialCommandGroup(
                        Commands.runOnce(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage)),
                        new MoveArm(arm, armIntake, ArmSetpoints.GROUND_INTAKE_CONE)));
        Constants.AutoConstants.eventMap.put("Sting", new MoveArm(arm, armIntake, ArmSetpoints.STING));
        Constants.AutoConstants.eventMap.put("Stow",
                new SequentialCommandGroup(
                        Commands.runOnce(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage)),
                        new MoveArm(arm, armIntake, ArmSetpoints.STOW)));
        Constants.AutoConstants.eventMap.put("Place Mid", new MoveArm(arm, armIntake, ArmSetpoints.PLACE_MID));

        Constants.AutoConstants.eventMap.put("AutoBalance", new AutoBalancingPID(swerve));
        Constants.AutoConstants.eventMap.put("Place Top", new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP));
        Constants.AutoConstants.eventMap.put("Wait", new WaitCommand(5));

    }
}