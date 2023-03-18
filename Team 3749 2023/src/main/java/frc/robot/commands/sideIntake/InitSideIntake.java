package frc.robot.commands.sideIntake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.SideIntake;
import frc.robot.utils.Constants;

/**
 * Move the side out of its starting position and back
 * 
 * @author Rohin Sood
 */
public class InitSideIntake extends SequentialCommandGroup {
    
    public InitSideIntake(SideIntake sideIntake) {
        addRequirements(sideIntake);

        addCommands(
            new LiftSideIntake(sideIntake, Constants.SideIntake.liftOutSetpoint / 2),
            new LiftSideIntake(sideIntake, 0.0)
        );
    }

}
