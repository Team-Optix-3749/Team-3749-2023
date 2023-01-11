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
    private CANSparkMax neo_motor1 = new CANSparkMax(Constants.Arm.neo_motor1_port, MotorType.kBrushless); // Check if this is actually brushless later
    private CANSparkMax neo_motor2 = new CANSparkMax(Constants.Arm.neo_motor2_port, MotorType.kBrushless); // Check if this is actually brushless later

    public Arm() {

    }

    
    
}
