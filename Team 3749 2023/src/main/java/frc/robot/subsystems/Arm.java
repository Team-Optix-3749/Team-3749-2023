package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/*
 * @author Bailey Say
 * @author Raymond Sheng
 * 
 * Code for the arm of the robot. This does not include the claw at the end
 * 
 * Naming convention:
 * neo_motor_lower 1 and 2 refer to bottom two motors controlling the lower joint
 * neo_motor_upper 1 and 2 refer to upper two motors controlling the upper joint
 */

public class Arm extends SubsystemBase {
    private CANSparkMax neo_motor_lower1 = new CANSparkMax(Constants.Arm.neo_motor_lower_id_1, MotorType.kBrushless); // Check if this is actually brushless later
    private CANSparkMax neo_motor_lower2 = new CANSparkMax(Constants.Arm.neo_motor_lower_id_2, MotorType.kBrushless); // Check if this is actually brushless later
    private CANSparkMax neo_motor_upper1 = new CANSparkMax(Constants.Arm.neo_motor_upper_id_1, MotorType.kBrushless); // Check if this is actually brushless later
    private CANSparkMax neo_motor_upper2 = new CANSparkMax(Constants.Arm.neo_motor_upper_id_2, MotorType.kBrushless); // Check if this is actually brushless later

    private MotorControllerGroup upperMotorControllerGroup = new MotorControllerGroup(neo_motor_upper1, neo_motor_upper2, null);
    private MotorControllerGroup lowerMotorControllerGroup = new MotorControllerGroup(neo_motor_lower1, neo_motor_lower2, null);
        
    // Standard classes for controlling our arm
    // Not sure if same values for k values from Constants.Arm should be used for PIDs of upper and lower. Check this later
    private final PIDController topController = new PIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd);
    private final PIDController bottomController = new PIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd);
    
    // Relative Encoders
    private final RelativeEncoder lowerEncoder1 = neo_motor_lower1.getEncoder(); 
    private final RelativeEncoder lowerEncoder2 = neo_motor_lower2.getEncoder();
    private final RelativeEncoder upperEncoder1 = neo_motor_upper1.getEncoder(); 
    private final RelativeEncoder upperEncoder2 = neo_motor_upper2.getEncoder();

    // Actual Arm Code
    public Arm() {
        neo_motor_lower2.setInverted(true);
        neo_motor_upper2.setInverted(true);
    }

    // Sets speed of a motor controller group
    public void setSpeedUpper(double speed) {
        upperMotorControllerGroup.set(speed); // says "fix" here but not sure what it's referring to
    }

    public void setSpeedLower(double speed) {
        lowerMotorControllerGroup.set(speed); // says "fix" here but not sure what it's referring to
    }

    // PID + feedforward implementation; should return the needed voltage
    // also add calculateLower()
    public double calculateUpper() {
        // upperMotorControllerGroup.set(topController.calculate(upperEncoder1.getDistance(), setpoint));//taken from wpilib documentation: not too sure how this all works yet       
        return 0;
    }

    public double calculateLower() {
        // lowerMotorControllerGroup.set(bottomController.calculate(lowerEncoder1.getDistance(), setpoint));
        return 0;
    }

}