package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;
import frc.robot.commands.ClawOuttakeCommand;
import frc.robot.subsystems.Claw;
import frc.robot.commands.ClawIntakeCommand;

public class RobotContainer {

    // Controllers
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);

    private final POV pilotPOV = new POV(pilot);
    private final POV operatorPOV = new POV(operator);

    private final Claw claw = new Claw();

    // Subsystems

    // Commands

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureDefaultCommands() {
    }

    private void configureButtonBindings() {
        pilot.a().whileTrue(new ClawIntakeCommand(claw)); // while a is held, intake
        pilot.b().whileTrue(new ClawOuttakeCommand(claw)); // while b is held, outtake
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
