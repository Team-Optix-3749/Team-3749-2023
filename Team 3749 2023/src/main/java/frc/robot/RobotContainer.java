package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.utils.*;
import frc.robot.Constants.*;

public class RobotContainer {
    // Controllers
    private final Xbox pilot = new Xbox(OIConstants.kPilotControllerPort);
    // private final Xbox operator = new Xbox(OIConstants.kOperatorControllerPort);

    // Subsystems
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public RobotContainer() {
        setupAuto();
        configureButtonBindings();
        configureDefaultCommands();

        swerveSubsystem.setDefaultCommand(new SwerveTeleopCommand(
                swerveSubsystem,
                () -> -pilot.getLeftY(),
                () -> pilot.getLeftX(),
                () -> pilot.getRightX()));

        configureButtonBindings();
    }

    /**
     * Set default commands
     */
    private void configureDefaultCommands() {
    }

    /**
     * Set controller button bindings
     */
    private void configureButtonBindings() {
        pilot.aWhileHeld(() -> swerveSubsystem.zeroHeading(), swerveSubsystem);
    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        return AutoCommands.getTestPathPlanner(swerveSubsystem, Alliance.Blue);
    }

    /**
     * Set event maps for autonomous
     */
    public void setupAuto() {
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
        // Constants.AutoConstants.eventMap.put("run_claw", Commands.run(() -> clawSubsystem.set(0.2), clawSubsystem));
    }
}
