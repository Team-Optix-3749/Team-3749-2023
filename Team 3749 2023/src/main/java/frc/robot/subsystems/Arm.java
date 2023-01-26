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
 * @author Don Tran
 * 
 * Code for the arm of the robot. This does not include the claw at the end
 * 
 */

public class Arm extends SubsystemBase {
    private CANSparkMax leftShoulderMotor = new CANSparkMax(Constants.Arm.left_shoulder_id, MotorType.kBrushless);
    private CANSparkMax rightShoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);
    private CANSparkMax leftElbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private CANSparkMax rightElbowMotor = new CANSparkMax(Constants.Arm.right_elbow_id, MotorType.kBrushless);

    // Not sure of values for kp, ki, kd
    private final PIDController shoulderController = new PIDController(Constants.Arm.kp, Constants.Arm.ki,
            Constants.Arm.kd);
    private final PIDController elbowController = new PIDController(Constants.Arm.kp, Constants.Arm.ki,
            Constants.Arm.kd);

    // Relative Encoder Initialization
    private final RelativeEncoder leftShoulderEncoder = leftShoulderMotor.getEncoder();
    private final RelativeEncoder rightShoulderEncoder = rightShoulderMotor.getEncoder();
    private final RelativeEncoder leftElbowEncoder = leftElbowMotor.getEncoder();
    private final RelativeEncoder rightElbowEncoder = rightElbowMotor.getEncoder();

    public Arm() {
        // invert right motors
        rightElbowMotor.setInverted(true);
        rightShoulderMotor.setInverted(true);

        // right motors are follower motors for left motors
        rightShoulderMotor.follow(leftShoulderMotor);
        rightElbowMotor.follow(leftElbowMotor);

        // conversion factor is ((gear ratio)/(encoder resolution) * 360) degrees
        leftShoulderEncoder.setPositionConversionFactor(250/2048*360);
        leftElbowEncoder.setPositionConversionFactor(250/2048*36);
    }

    // Sets speed of a motor controller group
    public void setSpeedElbow(double speed) {
        leftElbowMotor.set(speed);
    }

    public void setSpeedShoulder(double speed) {
        leftShoulderMotor.set(speed);
    }

    /* PID + feedforward implementation; should return the needed voltage, need to
    // do feedforward
    // desired posiiton and make sure position values are good for both
    public void setForearmVoltage(double x, double y) {

    // this sets voltage to degrees (not good)
         pperMotorControllerGroup.setVoltage(
                 forearmController.calculate(leftForearmEncoder.getPosition(),
                         BruteInverseKinematics.calculate(x, y)[0]));
    // taken from wpilib documentation: not too sure how this all works yet
    }
    */

    // public void setBicepVoltage(double x, double y) {

    //     // this sets voltage to degrees (not good)
    //     lowerMotorControllerGroup.setVoltage(
    //             bicepController.calculate(leftBicepEncoder.getPosition(), BruteInverseKinematics.calculate(x, y)[1]));
    //     // index idk if we want to clean this up lmao
    // }

    public void setDegreesShoulder(double x, double y) {
        // NEED PID TO CONTROL ACCURATELY
        // would work if the conversion factor was correctly set
        // need to change where it gets the calculated angle (x and y vals)
        leftShoulderEncoder.setPosition(shoulderController.calculate(leftShoulderEncoder.getPosition(), BruteInverseKinematics.calculate(x, y)[1]));
    }

    public void setDegreesElbow(double x, double y) {
        // NEED PID TO CONTROL ACCURATELY
        // would work if the conversion factor was correctly set
        // need to change where it gets the calculated angle (x and y vals)
        leftElbowEncoder.setPosition(elbowController.calculate(leftElbowEncoder.getPosition(), BruteInverseKinematics.calculate(x, y)[1]));
    }
}