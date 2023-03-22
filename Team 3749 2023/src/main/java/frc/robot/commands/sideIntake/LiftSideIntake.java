package frc.robot.commands.sideIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.SideIntake;

/**
 * Lift the side intake and stop when at its setpoint
 * 
 * @author Rohin Sood
 */
public class LiftSideIntake extends CommandBase {
    private SideIntake sideIntake;
    private double setpoint;

    public LiftSideIntake(SideIntake sideIntake, double setpoint) {
        this.sideIntake = sideIntake;
        this.setpoint = setpoint;
        addRequirements(sideIntake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        sideIntake.setLiftPIDFF(this.setpoint);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return sideIntake.liftAtSetpoint();
    }
}
