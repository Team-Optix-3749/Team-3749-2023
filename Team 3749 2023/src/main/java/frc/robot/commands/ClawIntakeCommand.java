package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Anusha Khobare
 * @author Aashray Reddy
 * @author Ryan R McWeeny
 * @author Hanlun Li
 * 
 *     ClawOuttake.Java is a comand that runs the claw motors backwards causing it to intake objects (dependent on Claw.Java and Constants.java)
 */
public class ClawIntakeCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    Claw claw; //adds claw object

    // Initializes the ClawCommand
    public ClawIntakeCommand(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    // Run on command init
    @Override
    public void initialize() {
    }

    // Run every 20 ms
    @Override
    public void execute() {
        // uses PID to calculate the velocity needed to acheive an exact speed
        claw.setSpeed(-Constants.Claw.setpoint_velocity);
        
    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        claw.setSpeed(Constants.Claw.stop); //set speed to 0 (stop)
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}