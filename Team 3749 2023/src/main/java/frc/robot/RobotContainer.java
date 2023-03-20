package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.intake.*;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.swerve.AutoBalancingPID;
import frc.robot.commands.swerve.AutoCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.AutoConstants.TopBottom;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);

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
    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveTeleopCommand(
                swerve,
                () -> -pilot.getLeftY(),
                () -> pilot.getLeftX(),
                () -> pilot.getRightX()));

        sideIntake.setDefaultCommand(
                Commands.run(() -> sideIntake.setIntakeVoltage(Constants.SideIntake.idleVoltage), sideIntake));
    }

    /**
     * Set controller button bindings
     * 
     */
    private void configureButtonBindings() {
        // arm setpoints (buttons)
        pilot.a().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP));
        pilot.b().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.PLACE_MID));
        pilot.x().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.GROUND_INTAKE_CUBE));
        pilot.y().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.GROUND_INTAKE_CONE));

        // arm setpoints (bumpers)
        pilot.rightBumper().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.STING));
        pilot.leftBumper().onTrue(new MoveArm(arm, armIntake, ArmSetpoints.DOUBLE_SUBSTATION));

        // intake button bindings
        pilot.rightTriggerWhileHeld(Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.releaseConeVoltage)),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage), armIntake));
        pilot.leftTriggerWhileHeld(Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.intakeVoltage)),
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage), armIntake));

        // swerve button bindings
        pilot.backWhileHeld(() -> swerve.resetGyro(), swerve);

    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        return AutoCommands.getAlexHouse(swerve, arm, armIntake, TopBottom.TOP);
        // return AutoCommands.getTwoPiece(swerve, arm, armIntake,
        //         TopBottom.TOP);
        // return AutoCommands.getThreePiece(swerve, arm, armIntake,
        //         TopBottom.TOP);

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
        Constants.AutoConstants.eventMap.put("AutoBalance", new AutoBalancingPID(swerve));
        Constants.AutoConstants.eventMap.put("Place Top", new MoveArm(arm, armIntake, ArmSetpoints.PLACE_TOP));
        Constants.AutoConstants.eventMap.put("Wait", new WaitCommand(5));

    }
}