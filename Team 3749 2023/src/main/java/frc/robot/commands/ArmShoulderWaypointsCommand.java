package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;

public class ArmShoulderWaypointsCommand extends SequentialCommandGroup {
    public ArmShoulderWaypointsCommand(Arm arm, double angle1, double angle2) {
        addCommands(
            new InstantCommand(() -> arm.setShoulderAngle(angle1)),
            new WaitUntilCommand(() -> arm.getShoulderAtSetpoint(angle1)),
            new InstantCommand(() -> arm.setShoulderAngle(angle2)),
            new WaitUntilCommand(() -> arm.getShoulderAtSetpoint(angle2))
        );
    }
    
}
