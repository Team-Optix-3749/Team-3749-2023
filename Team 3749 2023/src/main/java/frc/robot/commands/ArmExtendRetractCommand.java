package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants;

/***
 * 
 * @author Don Tran
 */

public class ArmExtendRetractCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    Arm arm = new Arm();

    // Initializes the BaseCommand
    public ArmExtendRetractCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // Run on command init
    @Override
    public void initialize() {
        arm.setSpeedTelescope(Constants.Arm.neo_motor_telescope_speed);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            end(isFinished());
        }
    }

    @Override
    public void execute() {
    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        arm.setSpeedTelescope(Constants.Arm.neo_motor_telescope_stop);
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}
