package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.BruteInverseKinematics;

/**
 * @author Bailey Say
 * @author Raymond Sheng
 * 
 * Code for the arm of the robot. This does not include the claw at the end
 * 
 * 
 */

public class Arm extends SubsystemBase {
    private CANSparkMax leftBicepMotor = new CANSparkMax(Constants.Arm.left_bicep_id, MotorType.kBrushless);
    private CANSparkMax rightBicepMotor = new CANSparkMax(Constants.Arm.right_bicep_id, MotorType.kBrushless);
    private CANSparkMax leftForearmMotor = new CANSparkMax(Constants.Arm.left_forearm_id, MotorType.kBrushless);
    private CANSparkMax rightForearmMotor = new CANSparkMax(Constants.Arm.right_forearm_id, MotorType.kBrushless);

    // set leaders and followers using .follow() in the constructor so you can
    // control the encoders of both motor controllers
    // private MotorControllerGroup upperMotorControllerGroup = new MotorControllerGroup(leftBicepMotor, rightBicepMotor,
    //         null);
    // private MotorControllerGroup lowerMotorControllerGroup = new MotorControllerGroup(leftForearmMotor,
    //         rightForearmMotor, null);

    // Standard classes for controlling our arm
    // Not sure if same values for k values from Constants.Arm should be used for
    // PIDs of upper and lower. Check this later
    private final PIDController bicepController = new PIDController(Constants.Arm.kp, Constants.Arm.ki,
            Constants.Arm.kd);
    private final PIDController forearmController = new PIDController(Constants.Arm.kp, Constants.Arm.ki,
            Constants.Arm.kd);

    // Relative Encoders (dk if we need all of these)
    private final RelativeEncoder leftBicepEncoder = leftForearmMotor.getEncoder();
    private final RelativeEncoder rightBicepEncoder = rightForearmMotor.getEncoder();
    private final RelativeEncoder leftForearmEncoder = leftBicepMotor.getEncoder();
    private final RelativeEncoder rightForearmEncoder = rightBicepMotor.getEncoder();

    public Arm() {
        // inverted right motors
        rightForearmMotor.setInverted(true);
        rightBicepMotor.setInverted(true);

        // right motors are follower motors for left motors
        rightBicepMotor.follow(leftBicepMotor);
        rightForearmMotor.follow(leftForearmMotor);

        // conversion factor is ((gear ratio)/(encoder resolution) * 360) degrees
        leftBicepEncoder.setPositionConversionFactor(250/2048*360);
        leftForearmEncoder.setPositionConversionFactor(250/2048*36);
    }

    // Sets speed of a motor controller group (dk if we neeed these)
    public void setSpeedUpper(double speed) {
        leftForearmMotor.set(speed);
    }

    public void setSpeedLower(double speed) {
        leftBicepMotor.set(speed);
    }

    // PID + feedforward implementation; should return the needed voltage, need to
    // do feedforward
    // desired posiiton and make sure position values are good for both
    // public void setForearmVoltage(double x, double y) {

    //     // this sets voltage to degrees (not good)
    //     upperMotorControllerGroup.setVoltage(
    //             forearmController.calculate(leftForearmEncoder.getPosition(),
    //                     BruteInverseKinematics.calculate(x, y)[0]));
    //     // taken from wpilib documentation: not too sure how this all works yet
    // }

    // public void setBicepVoltage(double x, double y) {

    //     // this sets voltage to degrees (not good)
    //     lowerMotorControllerGroup.setVoltage(
    //             bicepController.calculate(leftBicepEncoder.getPosition(), BruteInverseKinematics.calculate(x, y)[1]));
    //     // index idk if we want to clean this up lmao
    // }

    // public void setDegreesUpper(double anlge) {
    //     // NEED PID TO CONTROL ACCURATELY
    //     // would work if the conversion factor was correctly set
    //     leftBicepEncoder.setPosition(anlge);
    // }

    public void setForearmPosition(){

    }

    public void setBicepPosition(){

    }
}