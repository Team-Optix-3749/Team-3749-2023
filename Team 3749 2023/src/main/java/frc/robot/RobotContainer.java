package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.*;
import frc.robot.commands.swerve.AutoCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.utils.*;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);

    private final Swerve swerve = new Swerve();

    private final JoystickIO joystickIO = new JoystickIO(pilot, swerve);

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

    }

    /**
     * Set controller button bindings
     */
    public void configureButtonBindings() {

        if (!JoystickIO.didJoysticksChange())
            return;
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        joystickIO.pilotBindings();

    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        return AutoCommands.getTopTaxi(swerve);
    }

    /**
     * Set event maps for autonomous
     */
    public void configureAuto() {

    }

}