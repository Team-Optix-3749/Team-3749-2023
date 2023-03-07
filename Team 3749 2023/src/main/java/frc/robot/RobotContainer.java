package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.claw.*;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.swerve.AutoCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

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
            Commands.run(() -> claw.setVoltage(Constants.Claw.idleVoltage), claw)
        );

        // arm.setDefaultCommand(
        //     new MoveArm(arm, claw, ArmSetpoints.CONE_TOP)
        // );
    }

    /**
     * Set controller button bindings
     * 
     */
    private void configureButtonBindings() {
        // arm setpoints (buttons)
        pilot.a().onTrue(new MoveArm(arm, claw, ArmSetpoints.PLACE_TOP));
        pilot.b().onTrue(new MoveArm(arm, claw, ArmSetpoints.PLACE_MID));
        pilot.x().onTrue(new MoveArm(arm, claw, ArmSetpoints.GROUND_INTAKE));

        // arm setpoints (bumpers)
        pilot.rightBumper().onTrue(new MoveArm(arm, claw, ArmSetpoints.STING));
        pilot.leftBumper().onTrue(new MoveArm(arm, claw, ArmSetpoints.DOUBLE_SUBSTATION));
        
        // intake button bindings
        pilot.rightTriggerWhileHeld(() -> claw.setVoltage(Constants.Claw.releaseObjectVoltage));
        pilot.leftTriggerWhileHeld(() -> claw.setVoltage(Constants.Claw.intakeVoltage));
        
        // swerve button bindings
        pilot.backWhileHeld(() -> swerve.zeroHeading(), swerve);

        // swerve rotation cardinals
        pilot.povUp().whileTrue(Commands.run(() -> swerve.turnToRotation(0)));
        pilot.povLeft().whileTrue(Commands.run(() -> swerve.turnToRotation(270)));
        pilot.povDown().whileTrue(Commands.run(() -> swerve.turnToRotation(180)));
        pilot.povRight().whileTrue(Commands.run(() -> swerve.turnToRotation(90)));
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