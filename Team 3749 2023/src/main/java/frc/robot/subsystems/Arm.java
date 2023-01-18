package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/*
 * @author Bailey Say
 * 
 * Code for the arm of the robot. This does not include the claw at the end
 */

public class Arm extends SubsystemBase {
    

    // Someone fix this
    private CANSparkMax neo_motor_lower_left = new CANSparkMax(Constants.Arm.neo_motor_lower_left_port, MotorType.kBrushless); // Check if this is actually brushless later
    private CANSparkMax neo_motor_lower_right = new CANSparkMax(Constants.Arm.neo_motor_lower_right_port, MotorType.kBrushless); // Check if this is actually brushless later

    private final DCMotor armGearbox = DCMotor.getNEO(Constants.Arm.number_of_motors);
        
    // Standard classes for controlling our arm
    private final ProfiledPIDController topController = new ProfiledPIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd, new TrapezoidProfile.Constraints(Constants.Arm.max_velocity, Constants.Arm.max_acceleration));
    private final ProfiledPIDController bottomController = new ProfiledPIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd, new TrapezoidProfile.Constraints(Constants.Arm.max_velocity, Constants.Arm.max_acceleration));
    
    // Relative Encoders
    private final RelativeEncoder topEncoder = neo_motor_lower_left.getEncoder(); // change
    private final RelativeEncoder bottomEncoder = neo_motor_lower_right.getEncoder();

    // Simulation Code
    private final SingleJointedArmSim arm_top_sim = new SingleJointedArmSim(armGearbox, Constants.Simulation.arm_reduction, SingleJointedArmSim.estimateMOI(Constants.Simulation.arm_top_length, Constants.Simulation.arm_top_mass),
            Constants.Simulation.arm_top_length,
            Units.degreesToRadians(Constants.Simulation.arm_top_min_angle),
            Units.degreesToRadians(Constants.Simulation.arm_top_max_angle),
            Constants.Simulation.arm_top_mass,
            false,
            VecBuilder.fill(Constants.Simulation.arm_encoder_dist_per_pulse)); // Add noise with a std-dev of 1 tick);

    // Actual Arm Code
    public Arm() {}

    // Sets speed of a motor
    // Pass in a motor and a speed to set that motor's speed
    public void setSpeed(CANSparkMax motor, double speed) {
        motor.set(speed); // says "fix" here but not sure what it's referring to
    }

    public CANSparkMax getMotorLowerLeft() {
        return this.neo_motor_lower_left;
    }

    public CANSparkMax getMotorLowerRight() {
        return this.neo_motor_lower_right;
    }

}