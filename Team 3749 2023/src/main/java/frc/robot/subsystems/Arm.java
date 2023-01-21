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
    private CANSparkMax leftshoulderMotor = new CANSparkMax(Constants.Arm.left_shoulder_id, MotorType.kBrushless);
    private CANSparkMax rightshoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);
    private CANSparkMax leftelbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private CANSparkMax rightelbowMotor = new CANSparkMax(Constants.Arm.right_elbow_id, MotorType.kBrushless);

    // set leaders and followers using .follow() in the constructor so you can
    // control the encoders of both motor controllers
    private MotorControllerGroup upperMotorControllerGroup = new MotorControllerGroup(leftshoulderMotor, rightshoulderMotor,
            null);
    private MotorControllerGroup lowerMotorControllerGroup = new MotorControllerGroup(leftelbowMotor,
            rightelbowMotor, null);

    // Standard classes for controlling our arm
    // Not sure if same values for k values from Constants.Arm should be used for
    // PIDs of upper and lower. Check this later
    private final PIDController shoulderController = new PIDController(Constants.Arm.kp, Constants.Arm.ki,
            Constants.Arm.kd);
    private final PIDController elbowController = new PIDController(Constants.Arm.kp, Constants.Arm.ki,
            Constants.Arm.kd);

    // Relative Encoders (dk if we need all of these)
    private final RelativeEncoder leftshoulderEncoder = leftelbowMotor.getEncoder();
    private final RelativeEncoder rightshoulderEncoder = rightelbowMotor.getEncoder();
    private final RelativeEncoder leftelbowEncoder = leftshoulderMotor.getEncoder();
    private final RelativeEncoder rightelbowEncoder = rightshoulderMotor.getEncoder();

    public Arm() {
        rightelbowMotor.setInverted(true);
        rightshoulderMotor.setInverted(true);

        // conversion factor is ((1/(gear reduction)) * (2 * Math.pi))
        leftshoulderEncoder.setPositionConversionFactor(0);
    }

    // Sets speed of a motor controller group (dk if we neeed these)
    public void setSpeedUpper(double speed) {
        upperMotorControllerGroup.set(speed);
    }

    public void setSpeedLower(double speed) {
        lowerMotorControllerGroup.set(speed);
    }

    // PID + feedforward implementation; should return the needed voltage, need to
    // do feedforward
    // desired posiiton and make sure position values are good for both
    public void setelbowVoltage(double x, double y) {

        // this sets voltage to degrees (not good)
        upperMotorControllerGroup.setVoltage(
                elbowController.calculate(leftelbowEncoder.getPosition(),
                        BruteInverseKinematics.calculate(x, y)[0]));
        // taken from wpilib documentation: not too sure how this all works yet
    }

    public void setshoulderVoltage(double x, double y) {

        // this sets voltage to degrees (not good)
        lowerMotorControllerGroup.setVoltage(
                shoulderController.calculate(leftshoulderEncoder.getPosition(), BruteInverseKinematics.calculate(x, y)[1]));
        // index idk if we want to clean this up lmao
    }

    public void setDegreesUpper(double anlge) {
        // NEED PID TO CONTROL ACCURATELY
        // would work if the conversion factor was correctly set
        leftshoulderEncoder.setPosition(anlge);
    }
}