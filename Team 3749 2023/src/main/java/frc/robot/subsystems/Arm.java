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
    private CANSparkMax neoMotorTelescope = new CANSparkMax(Constants.Arm.neo_motor_telescope_port, MotorType.kBrushless); // Check if this is actually brushless later
    private CANSparkMax neoMotorElevator = new CANSparkMax(Constants.Arm.neo_motor_elevator_port, MotorType.kBrushless); // Check if this is actually brushless later

    public Arm() {

    }

    public void setSpeedTelescope(double speed) {
        neoMotorTelescope.set(speed);
    }

    public void setSpeedElevator(double speed) {
        neoMotorElevator.set(speed);
    }

    
}
