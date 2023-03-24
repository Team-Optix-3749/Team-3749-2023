package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.swerve.AutoBalancingPID;
import frc.robot.commands.vision.AlignApriltag;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.ArmIntake;
import frc.robot.subsystems.intake.SideIntake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.LEDs.LEDPattern;

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
    private SideIntake sideIntake;
    private Arm arm;

    public JoystickIO(Xbox pilot, Xbox operator, Swerve swerve, Limelight limelight, LEDs leds, ArmIntake armIntake,
            SideIntake sideIntake, Arm arm) {
        this.pilot = pilot;
        this.limelight = limelight;
        this.operator = operator;
        this.swerve = swerve;
        this.limelight = limelight;
        this.leds = leds;
        this.armIntake = armIntake;
        this.sideIntake = sideIntake;
        this.arm = arm;
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
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public void pilotAndOperatorBindings() {
        // arm setpoints (buttons)
        operator.a().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.PLACE_TOP));
        operator.b().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.PLACE_MID));
        operator.x().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.GROUND_INTAKE_CUBE));
        operator.y().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.GROUND_INTAKE_CONE));

        // arm setpoints (bumpers)
        operator.rightBumper().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.STING));
        operator.leftBumper().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.DOUBLE_SUBSTATION));

        operator.rightTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage));
        operator.leftTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage));

        operator.leftStickWhileHeld(() -> leds.setLEDPattern(LEDPattern.PURPLE), leds);
        operator.rightStickWhileHeld(() -> leds.setLEDPattern(LEDPattern.YELLOW), leds);

        // alignment (vision)
        operator.povUp().whileTrue(new AlignApriltag(swerve, limelight));
        operator.povLeft().whileTrue(new AlignApriltag(swerve, limelight, true));
        operator.povRight().whileTrue(new AlignApriltag(swerve, limelight, false));

        pilot.yWhileHeld(Commands.runOnce(() -> sideIntake.toggleLiftSetpoint(), sideIntake));
        pilot.a().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.GROUND_INTAKE_CUBE));
        pilot.xWhileHeld(new AlignApriltag(swerve, limelight, true));
        pilot.bWhileHeld(new AlignApriltag(swerve, limelight, false));

        pilot.rightBumperWhileHeld(() -> sideIntake.setIntakeVoltage(Constants.ArmIntake.releaseConeVoltage),
                sideIntake);
        pilot.leftBumperWhileHeld(() -> sideIntake.setIntakeVoltage(Constants.ArmIntake.intakeVoltage),
                sideIntake);

        pilot.leftTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage));
        pilot.rightTriggerWhileHeld(new AutoBalancingPID(swerve,0));  
        // swerve button bindings
        pilot.backWhileHeld(() -> swerve.resetGyro(), swerve);
        pilot.startWhileHeld(new AlignApriltag(swerve, limelight));

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
        // pilot.bWhileHeld(new AlignApriltag(swerve, limelight, false));
        pilot.a().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.PLACE_TOP));
        pilot.b().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.PLACE_MID));
        pilot.x().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.GROUND_INTAKE_CUBE));
        pilot.y().onTrue(Commands.runOnce(() -> sideIntake.toggleLiftSetpoint(), sideIntake));

        // arm setpoints (bumpers)
        pilot.rightBumper().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.STING));
        pilot.leftBumper().onTrue(new MoveArm(arm, armIntake, leds, ArmSetpoints.DOUBLE_SUBSTATION));

        // intake button bindings
        pilot.rightTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage),
                () -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage));
        pilot.leftTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage),
                () -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage));

        // swerve button bindings
        pilot.backWhileHeld(() -> swerve.resetGyro(), swerve);
        pilot.startWhileHeld(
                () -> swerve.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(swerve.getHeading()))),
                swerve);
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

        armCommands.add(new MoveArm(arm, armIntake, leds, ArmSetpoints.PLACE_TOP));
        armCommands.add(new MoveArm(arm, armIntake, leds, ArmSetpoints.PLACE_MID));
        armCommands.add(new MoveArm(arm, armIntake, leds, ArmSetpoints.GROUND_INTAKE_CUBE));
        armCommands.add(new MoveArm(arm, armIntake, leds, ArmSetpoints.STING));
        armCommands.add(new MoveArm(arm, armIntake, leds, ArmSetpoints.DOUBLE_SUBSTATION));

        ShuffleboardLayout sideIntakeCommands = controlsTab
                .getLayout("Side Intake", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN"));

        CommandBase liftSideIntake = Commands.runOnce(() -> sideIntake.toggleLiftSetpoint(), sideIntake);
        liftSideIntake.setName("Toggle Lift");
        CommandBase outakeSideIntake = Commands
                .run(() -> sideIntake.setIntakeVoltage(Constants.ArmIntake.releaseConeVoltage), sideIntake);
        outakeSideIntake.setName("Side Outake");
        CommandBase intakeSideIntake = Commands
                .run(() -> sideIntake.setIntakeVoltage(Constants.ArmIntake.intakeVoltage), sideIntake);
        intakeSideIntake.setName("Side Intake");

        sideIntakeCommands.add(liftSideIntake);
        sideIntakeCommands.add(outakeSideIntake);
        sideIntakeCommands.add(intakeSideIntake);

        ShuffleboardLayout armIntakeCommands = controlsTab
                .getLayout("Arm Intake", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN"));

        CommandBase outakeArmIntake = Commands
                .run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage), sideIntake);
        outakeArmIntake.setName("Arm Outake");
        CommandBase intakeArmIntake = Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage),
                sideIntake);
        intakeArmIntake.setName("Arm Intake");

        armIntakeCommands.add(outakeArmIntake);
        armIntakeCommands.add(intakeArmIntake);
    }
}
