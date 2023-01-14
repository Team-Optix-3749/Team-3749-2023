package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/*
 * @author Bailey Say
 * 
 * Code for the arm of the robot. This does not include the claw at the end
 */

public class Arm extends SubsystemBase {
    
    private CANSparkMax neo_motor_lower = new CANSparkMax(Constants.Arm.neo_motor_lower_port, MotorType.kBrushless); // Check if this is actually brushless later
    private CANSparkMax neo_motor_upper = new CANSparkMax(Constants.Arm.neo_motor_upper_port, MotorType.kBrushless); // Check if this is actually brushless later

private int level = 0;

    public Arm() {}

    public void setSpeedLower(double speed) {
        neo_motor_lower.set(speed);
    }

    public void setSpeedUpper(double speed) {
        neo_motor_upper.set(speed);
    }

    public void setSpeedTelescope(double speed) {
        neo_motor_telescope.set(speed);
    }

    public boolean raiseLevel() {
        if (level < 4) {
            level += 1;
            return false;
        } //else {  We're already returning, the code ends there, however i'm leaving this here incase this leads to an error
            return true;
        //}
    }

    public boolean lowerLevel() {
        if (level > 0) {
            level -= 1;
            return false;
        } //else { Same reasoning as above 
            return true;
        //}
    
    }


    
    
}
