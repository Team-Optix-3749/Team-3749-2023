package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.swerve.AutoBalancingPID;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.vision.AlignApriltag;
import frc.robot.commands.vision.AlignPiece;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmTrajectories;
import frc.robot.subsystems.intake.ArmIntake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.VisionConstants.Piece;
import frc.robot.utils.Constants.VisionConstants.Pipelines;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 */
public class JoystickIO {
    private static String[] lastJoystickNames = new String[] { "", "", "", "", "", "" };

    private Xbox pilot;
    private Xbox operator;

    private Swerve swerve;
    private Limelight limelight;
    private LEDs leds;
    private ArmIntake armIntake;
    private Arm arm;
    private ArmTrajectories armTrajectories;

    public JoystickIO(Xbox pilot, Xbox operator) {
        this.pilot = pilot;
        this.limelight = Robot.limelight;
        this.operator = operator;
        this.swerve = Robot.swerve;
        this.limelight = Robot.limelight;
        this.leds = Robot.leds;
        this.armIntake = Robot.armIntake;
        this.arm = Robot.arm;
        this.armTrajectories = Arm.armTrajectories;
    }

    public static boolean didJoysticksChange() {
        boolean joysticksChanged = false;
        for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
            String name = DriverStation.getJoystickName(port);
            if (!name.equals(lastJoystickNames[port])) {
                joysticksChanged = true;
                lastJoystickNames[port] = name;
            }
        }
        return joysticksChanged;
    }

    /**
     * Calls binding methods according to the joysticks connected
     */
    public void getButtonBindings() {

        if (DriverStation.isJoystickConnected(1)) {
            // if both xbox controllers are connected
            pilotAndOperatorBindings();

        } else if (DriverStation.isJoystickConnected(0)) {
            // if only one xbox controller is connected
            pilotBindings();

        } else {
            // if no joysticks are connected (ShuffleBoard buttons)
            noJoystickBindings();
        }

        setDefaultCommands();
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public void pilotAndOperatorBindings() {
        // arm setpoints (buttons)
        operator.a().onTrue(new MoveArm(ArmSetpoints.PLACE_TOP));
        operator.b().onTrue(new MoveArm(ArmSetpoints.PLACE_MID));
        operator.x().onTrue(new MoveArm(ArmSetpoints.GROUND_INTAKE_CUBE));
        operator.y().onTrue(new MoveArm(ArmSetpoints.CUBE_STOW));

        // arm setpoints (bumpers)
        operator.rightBumper().onTrue(new MoveArm(ArmSetpoints.STING));
        operator.leftBumper()
                .onTrue(new MoveArm(ArmSetpoints.DOUBLE_SUBSTATION_CONE));
        operator.povUp()
                .onTrue(new MoveArm(ArmSetpoints.DOUBLE_SUBSTATION_CUBE));
        operator.povDown()
                .onTrue(new MoveArm(ArmSetpoints.DOUBLE_SUBSTATION_CONE));
        operator.povLeft().onTrue(
                new SequentialCommandGroup(
                        new MoveArm(ArmSetpoints.PLACE_TOP),
                        Commands.waitSeconds(0.75),
                        Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage))
                                .withTimeout(0.175),
                        Commands.runOnce(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage))
                                .withTimeout(0.175)));

        operator.rightTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage * 0.55),
                () -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage));
        operator.leftTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage),
                () -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage));

        operator.rightStickWhileHeld(Commands.runOnce(() -> arm.kill(), arm));

        // operator.leftStickWhileHeld(() -> leds.setLEDPattern(LEDPattern.PURPLE),
        // leds);
        // operator.rightStickWhileHeld(() -> leds.setLEDPattern(LEDPattern.YELLOW),
        // leds);

        pilot.a()
                .onTrue(new ParallelCommandGroup(
                        new MoveArm(ArmSetpoints.STING, true),

                        new AlignApriltag().withTimeout(2)));
        pilot.x()
                .onTrue(new ParallelCommandGroup(
                        new MoveArm(ArmSetpoints.STING, true),
                        new AlignApriltag(true).withTimeout(2)));
        pilot.b()
                .onTrue(new ParallelCommandGroup(
                        new MoveArm(ArmSetpoints.STING, true),
                        new AlignApriltag(false).withTimeout(2)));

        pilot.yWhileHeld(() -> swerve.toggleSpeed());

        pilot.leftTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage),
                () -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage));
        pilot.rightTriggerWhileHeld(new AutoBalancingPID(swerve, 0));
        // swerve button bindings
        pilot.startWhileHeld(Commands.runOnce(() -> {
            swerve.setFlipGyro(false);
            swerve.resetGyro();
        }, swerve));

        // swerve rotation cardinals
        pilot.povUp().whileTrue(Commands.run(() -> swerve.turnToRotation(0)));
        pilot.povLeft().whileTrue(Commands.run(() -> swerve.turnToRotation(270)));
        pilot.povDown().whileTrue(Commands.run(() -> swerve.turnToRotation(180)));
        pilot.povRight().whileTrue(Commands.run(() -> swerve.turnToRotation(90)));
    }

    /**
     * If only one controller is plugged in (pi)
     */
    public void pilotBindings() {
        // arm setpoints (buttons)
        // pilot.bWhileHeld(new AlignApriltag(false));
        pilot.a().onTrue(new MoveArm(ArmSetpoints.PLACE_TOP));
        pilot.b().onTrue(new MoveArm(ArmSetpoints.PLACE_MID));
        pilot.x().onTrue(new MoveArm(ArmSetpoints.GROUND_INTAKE_CUBE));

        // arm setpoints (bumpers)
        pilot.rightBumper().onTrue(new MoveArm(ArmSetpoints.STING));
        // pilot.leftBumper().onTrue(new MoveArm(
        // ArmSetpoints.DOUBLE_SUBSTATION));

        // intake button bindings
        pilot.rightTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage),
                () -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage));
        // pilot.leftTriggerWhileHeld(() ->
        // armIntake.setVoltage(Constants.ArmIntake.intakeVoltage),
        // () -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage));

        // swerve button bindings// pilot.startWhileHeld(
        // () -> swerve.resetOdometry(new Pose2d(new Translation2d(0, 0), new
        // Rotation2d(swerve.getHeading()))),
        // swerve);
        // swerve rotation cardinals
        pilot.povUp().whileTrue(Commands.run(() -> swerve.turnToRotation(0)));
        pilot.povLeft().whileTrue(Commands.run(() -> swerve.turnToRotation(270)));
        pilot.povDown().whileTrue(Commands.run(() -> swerve.turnToRotation(180)));
        pilot.povRight().whileTrue(Commands.run(() -> swerve.turnToRotation(90)));
    }

    /**
     * If NO joysticks are plugged in (Buttons for commands are runnable in the
     * "Controls" tab in ShuffleBoard)
     */
    public void noJoystickBindings() {
        ShuffleboardTab controlsTab = Shuffleboard.getTab("Controls");

        ShuffleboardLayout armCommands = controlsTab
                .getLayout("Arm", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        armCommands.add(new MoveArm(ArmSetpoints.PLACE_TOP));
        armCommands.add(new MoveArm(ArmSetpoints.PLACE_MID));
        armCommands.add(new MoveArm(ArmSetpoints.GROUND_INTAKE_CUBE));
        armCommands.add(new MoveArm(ArmSetpoints.STING));
        // armCommands.add(new MoveArm(
        // ArmSetpoints.DOUBLE_SUBSTATION));

        ShuffleboardLayout armIntakeCommands = controlsTab
                .getLayout("Arm Intake", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN"));

        CommandBase outakeArmIntake = Commands
                .run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage), armIntake);
        outakeArmIntake.setName("Arm Outake");
        CommandBase intakeArmIntake = Commands.run(() -> Robot.armIntake.setVoltage(Constants.ArmIntake.intakeVoltage),
                armIntake);
        intakeArmIntake.setName("Arm Intake");

        armIntakeCommands.add(outakeArmIntake);
        armIntakeCommands.add(intakeArmIntake);
    }

    /**
     * Sets the default commands
     */
    public void setDefaultCommands() {
        swerve.setDefaultCommand(new SwerveTeleopCommand(

                () -> -pilot.getLeftY(),
                () -> pilot.getLeftX(),
                () -> pilot.getRightX()));

        limelight.setDefaultCommand(
                Commands.run(
                        () -> {
                            limelight.setPipeline(Pipelines.APRILTAG.index);
                            limelight.updatePoseAprilTags(Robot.swerve.getPoseEstimator());
                        }, limelight));
    }
}
