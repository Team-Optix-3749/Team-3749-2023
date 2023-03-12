package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.intake.*;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.swerve.AutoCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);

    private static String[] lastJoystickNames = new String[] { "", "", "", "", "", "" };

    // Subsystems
    private final Swerve swerve = new Swerve();
    private final ArmIntake armIntake = new ArmIntake();
    private final SideIntake sideIntake = new SideIntake();
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
    public void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveTeleopCommand(
                swerve,
                () -> -pilot.getLeftY(),
                () -> pilot.getLeftX(),
                () -> pilot.getRightX()));

        armIntake.setDefaultCommand(
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage), armIntake));

        sideIntake.setDefaultCommand(
                Commands.run(() -> sideIntake.setIntakeVoltage(Constants.SideIntake.idleVoltage), sideIntake));
    }

    /**
     * Set controller button bindings
     */
    public void configureButtonBindings() {

        if (!didJoysticksChange())
            return;
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // if both xbox controllers are connected
        if (DriverStation.isJoystickConnected(1)) {

            // arm setpoints (buttons)
            operator.a().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP));
            operator.b().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.PLACE_MID));
            operator.x().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.GROUND_INTAKE));

            // arm setpoints (bumpers)
            operator.rightBumper().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.STING));
            operator.leftBumper().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.DOUBLE_SUBSTATION));

            operator.rightTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.releaseObjectVoltage));
            operator.leftTriggerWhileHeld(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage));

            pilot.y().onTrue(Commands.runOnce(() -> sideIntake.toggleLiftSetpoint(), sideIntake));

            pilot.rightTriggerWhileHeld(() -> sideIntake.setIntakeVoltage(Constants.ArmIntake.releaseObjectVoltage),
                    sideIntake);
            pilot.leftTriggerWhileHeld(() -> sideIntake.setIntakeVoltage(Constants.ArmIntake.intakeVoltage),
                    sideIntake);

            // swerve button bindings
            pilot.backWhileHeld(() -> swerve.zeroHeading(), swerve);

            // swerve rotation cardinals
            pilot.povUp().whileTrue(Commands.run(() -> swerve.turnToRotation(0)));
            pilot.povLeft().whileTrue(Commands.run(() -> swerve.turnToRotation(270)));
            pilot.povDown().whileTrue(Commands.run(() -> swerve.turnToRotation(180)));
            pilot.povRight().whileTrue(Commands.run(() -> swerve.turnToRotation(90)));

            // if only one xbox controller is connected
        } else if (DriverStation.isJoystickConnected(0)) {

            // arm setpoints (buttons)
            pilot.a().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP));
            pilot.b().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.PLACE_MID));
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

            // if no joysticks are connected (ShuffleBoard buttons)
        } else {

            ShuffleboardTab controlsTab = Shuffleboard.getTab("Controls");

            ShuffleboardLayout armCommands = controlsTab
                    .getLayout("Arm", BuiltInLayouts.kList)
                    .withSize(2, 2)
                    .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

            armCommands.add(new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP));
            armCommands.add(new MoveArm(arm, armIntake, ArmSetpoints.PLACE_MID));
            armCommands.add(new MoveArm(arm, armIntake, ArmSetpoints.GROUND_INTAKE));
            armCommands.add(new MoveArm(arm, armIntake, ArmSetpoints.STING));
            armCommands.add(new MoveArm(arm, armIntake, ArmSetpoints.DOUBLE_SUBSTATION));

            ShuffleboardLayout sideIntakeCommands = controlsTab
                    .getLayout("Side Intake", BuiltInLayouts.kList)
                    .withSize(2, 2)
                    .withProperties(Map.of("Label position", "HIDDEN"));

            CommandBase liftSideIntake = Commands.runOnce(() -> sideIntake.toggleLiftSetpoint(), sideIntake);
            liftSideIntake.setName("Toggle Lift");
            CommandBase outakeSideIntake = Commands
                    .run(() -> sideIntake.setIntakeVoltage(Constants.ArmIntake.releaseObjectVoltage), sideIntake);
            outakeSideIntake.setName("Side Outake");
            CommandBase intakeSideIntake = Commands
                    .run(() -> sideIntake.setIntakeVoltage(Constants.ArmIntake.intakeVoltage), sideIntake);
            intakeSideIntake.setName("Side Intake");

            sideIntakeCommands.add(liftSideIntake);
            sideIntakeCommands.add(outakeSideIntake);
            sideIntakeCommands.add(intakeSideIntake);

            ShuffleboardLayout armIntakeCommands = controlsTab
                    .getLayout("arm", BuiltInLayouts.kList)
                    .withSize(2, 2)
                    .withProperties(Map.of("Label position", "HIDDEN"));

            CommandBase outakeArmIntake = Commands
                    .run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseObjectVoltage), sideIntake);
            outakeArmIntake.setName("Arm Outake");
            CommandBase intakeArmIntake = Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage),
                    sideIntake);
            intakeArmIntake.setName("Arm Intake");

            armIntakeCommands.add(outakeArmIntake);
            armIntakeCommands.add(intakeArmIntake);

        }

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

}