package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/*
 * @author Bailey Say
 * 
 * Code for the arm of the robot. This does not include the claw at the end
 */

public class Arm extends SubsystemBase {
    private CANSparkMax lowerNeoMotor = new CANSparkMax(Constants.Arm.neo_motor_lower_port, MotorType.kBrushless); // Check if this is actually brushless later
    private CANSparkMax upperNeoMotor = new CANSparkMax(Constants.Arm.neo_motor_upper_port, MotorType.kBrushless); // Check if this is actually brushless later
    
    // level (temporary might change later)
    private int level = 0;

    private final DCMotor armGearbox = DCMotor.getNEO(Constants.Arm.number_of_motors);
        
    // Standard classes for controlling our arm
    private final ProfiledPIDController topController = new ProfiledPIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd, new TrapezoidProfile.Constraints(Constants.Arm.max_velocity, Constants.Arm.max_acceleration));
    private final ProfiledPIDController bottomController = new ProfiledPIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd, new TrapezoidProfile.Constraints(Constants.Arm.max_velocity, Constants.Arm.max_acceleration));
    
    // Relative Encoders
    private final RelativeEncoder topEncoder = upperNeoMotor.getEncoder();
    private final RelativeEncoder bottomEncoder = lowerNeoMotor.getEncoder();

    public Arm() {}

    public void setSpeedLower(double speed) {
        lowerNeoMotor.set(speed);
    }

    public void setSpeedUpper(double speed) {
        upperNeoMotor.set(speed);
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
