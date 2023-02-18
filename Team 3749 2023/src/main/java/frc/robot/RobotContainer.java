// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmSimCommand;
import frc.robot.commands.MoveArmHoldPID;
import frc.robot.commands.MoveArmPID;
import frc.robot.subsystems.arm.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Kinematics;
import frc.robot.utils.Xbox;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);

    // Subsystems
    private final Arm arm;

    public RobotContainer() {
        switch (Constants.ROBOT_MODE) {
            case REAL:
                arm = new ArmReal();
                break;
            case SIMULATION:
                arm = new ArmSim();
                break;
            default:
                arm = null;
                System.out.println("ROBOT_MODE is not set in utils/Constants.java");
                break;
        }

        try {
            configureDefaultCommands();
            configureButtonBindings();
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    private void configureDefaultCommands() throws Exception {
        switch (Constants.ROBOT_MODE) {
            case REAL:
                arm.setDefaultCommand(
                        Commands.run(() -> arm.setIdleMode(IdleMode.kBrake), arm));
                break;
            case SIMULATION:
                arm.setDefaultCommand(
                        new ArmSimCommand(arm));
            default:
                throw new Exception("ROBOT_MODE is not set in utils/Constants.java");
        }
    }

    private void configureButtonBindings() throws Exception {
        switch (Constants.ROBOT_MODE) {
            case REAL:
                pilot.aWhileHeld(
                        () -> arm.setShoulder(-.4), () -> arm.stopShoulder(), arm);
                pilot.bWhileHeld(
                        () -> arm.setShoulder(.4), () -> arm.stopShoulder(), arm);

                pilot.xWhileHeld(
                        () -> arm.setElbow(-.4), () -> arm.stopElbow(), arm);
                pilot.yWhileHeld(
                        () -> arm.setElbow(.4), () -> arm.stopElbow(), arm);

                pilot.rightBumper().whileTrue(new SequentialCommandGroup(
                        new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                                Constants.Arm.ElbowSetpoints.STING.angle),
                        new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.CUBETOP.angle,
                                Constants.Arm.ElbowSetpoints.CUBETOP.angle)));

                pilot.leftBumper().whileTrue(new SequentialCommandGroup(
                        new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                                Constants.Arm.ElbowSetpoints.STING.angle),
                        new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.STOWED.angle,
                                Constants.Arm.ElbowSetpoints.STOWED.angle)));
                break;
            case SIMULATION:
                // pilot.aWhileHeld(() -> testKinematics());
                break;
            default:
                throw new Exception("ROBOT_MODE is not set in utils/Constants.java");
        }
    }

    public void testKinematics() {
        try {
            Kinematics.tester();
        } catch (Exception e) {
            // TODO: handle exception
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
