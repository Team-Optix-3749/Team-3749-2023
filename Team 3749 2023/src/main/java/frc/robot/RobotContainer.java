package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.Constants;
import frc.robot.utils.POV;
import frc.robot.utils.Xbox;
import frc.robot.subsystems.Claw;

public class RobotContainer {

    // Controllers
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);

    // Subsystems
    private final Claw claw = new Claw();

    // Commands

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureDefaultCommands() {
    }

    private void configureButtonBindings() {
        pilot.aWhileHeld(
            () -> claw.set(Constants.Claw.speed.get()), () -> claw.set(0), claw
        );
        pilot.bWhileHeld(
            () -> claw.set(-Constants.Claw.speed.get()), () -> claw.set(0), claw
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
