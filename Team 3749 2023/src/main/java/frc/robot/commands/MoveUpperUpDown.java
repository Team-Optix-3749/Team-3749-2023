package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants;

/***
 * @author Aditya Saavedam
 * @author Don Tran
 */

public class MoveUpperUpDown extends CommandBase{
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private Arm arm;

    // Initializes the BaseCommand
    public MoveUpperUpDown(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // Run on command init
    @Override
    public void initialize() {
        arm.setSpeedUpper(Constants.Arm.neo_motor_upper_stop);
        arm.setSpeedLower(Constants.Arm.neo_motor_lower_stop);
    }

    @Override
    public void execute() {
        arm.setSpeedUpper(Constants.Arm.neo_motor_upper_speed); // basic set speed, calculate exact movement wednesday
    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        arm.setSpeedUpper(Constants.Arm.neo_motor_upper_stop);
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}
