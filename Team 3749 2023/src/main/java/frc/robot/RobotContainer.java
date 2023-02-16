package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Claw;
import frc.robot.commands.*;
import frc.robot.utils.*;
import frc.robot.Constants.*;

public class RobotContainer {
    // Controllers
    private final Xbox pilot = new Xbox(OIConstants.kDriverControllerPort);
    private final Xbox operator = new Xbox(1);

    // Subsystems
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Claw clawSubsystem = new Claw();

    // Commands
    private final MoveDistance moveDistance = new MoveDistance(swerveSubsystem, Units.feetToMeters(5));

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
        pilot.a().whileTrue(new InstantCommand(swerveSubsystem::zeroHeading));
        pilot.b().whileTrue(moveDistance);
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
        Constants.AutoConstants.eventMap.put("run_claw", Commands.run(() -> clawSubsystem.set(0.2), clawSubsystem));
    }
}
