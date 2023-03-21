package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.intake.*;
import frc.robot.commands.swerve.AutoCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.vision.VisionDefaultCommand;
import frc.robot.utils.*;
import frc.robot.utils.Constants;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);

    // Subsystems
    private final Swerve swerve = new Swerve();
    private final ArmIntake armIntake = new ArmIntake();
    private final SideIntake sideIntake = new SideIntake();
    private final Arm arm = new Arm();
    private final Limelight limelight = new Limelight();

    private final JoystickIO joystickIO = new JoystickIO(pilot, operator, swerve, limelight, armIntake, sideIntake, arm);

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

        armIntake.setDefaultCommand(
                Commands.run(() -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage), armIntake));

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

        joystickIO.getButtonBindings();

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