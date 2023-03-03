package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.*;
import frc.robot.commands.swerve.AutoCommands;
import frc.robot.commands.swerve.MoveToPose;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.utils.*;
import frc.robot.utils.Constants;

public class RobotContainer {
    // Controllers
    private final Xbox pilot = new Xbox(0);

    // Subsystems
    private final Swerve swerve = new Swerve();
    // private final Claw claw = new Claw();
    // private final Arm arm = new Arm();

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

        // claw.setDefaultCommand(
        //         Commands.run(() -> claw.setVoltage(1.0), claw));
    }

    /**
     * Set controller button bindings
     * 
     */
    private void configureButtonBindings() {
        pilot.aWhileHeld(
            new MoveToPose(swerve, new Pose2d(new Translation2d(15.48, 1.79), new Rotation2d(0.0)))
        );

        pilot.backWhileHeld(() -> swerve.zeroHeading(), swerve);
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