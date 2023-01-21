package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.InverseKinematics;

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
    
    // Relative Encoders (dk if we need all of these)
    private final RelativeEncoder lowerEncoder1 = neo_motor_lower1.getEncoder(); 
    private final RelativeEncoder lowerEncoder2 = neo_motor_lower2.getEncoder();
    private final RelativeEncoder upperEncoder1 = neo_motor_upper1.getEncoder(); 
    private final RelativeEncoder upperEncoder2 = neo_motor_upper2.getEncoder();

    public Arm() {
        neo_motor_lower2.setInverted(true);
        neo_motor_upper2.setInverted(true);

        lowerEncoder1.setPositionConversionFactor(0);
    }

    // Sets speed of a motor controller group (dk if we neeed these)
    public void setSpeedUpper(double speed) {
        upperMotorControllerGroup.set(speed); 
    }

    public void setSpeedLower(double speed) {
        lowerMotorControllerGroup.set(speed);
    }

    // PID + feedforward implementation; should return the needed voltage, need to do feedforward
    // desired posiiton and make sure position values are good for both
    public void setVoltageUpper(double x, double y) { 
        upperMotorControllerGroup.setVoltage(topController.calculate(upperEncoder1.getPosition(), InverseKinematics.calculate(x, y)[0]));//taken from wpilib documentation: not too sure how this all works yet
    }

    public void setVoltageLower(double x, double y) {
        lowerMotorControllerGroup.setVoltage(bottomController.calculate(lowerEncoder1.getPosition(), InverseKinematics.calculate(x, y)[1])); // index idk if we want to clean this up lmao
    }

    public void setDegreesUpper(double anlge) {
        lowerEncoder1.setPosition(anlge);

    }
}