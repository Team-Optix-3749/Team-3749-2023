package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.BruteInverseKinematics;

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
    private CANSparkMax leftBicepMotor = new CANSparkMax(Constants.Arm.left_bicep_id, MotorType.kBrushless); // Check if
                                                                                                             // this is
                                                                                                             // actually
                                                                                                             // brushless
                                                                                                             // later
    private CANSparkMax rightBicepMotor = new CANSparkMax(Constants.Arm.right_bicep_id, MotorType.kBrushless); // Check
                                                                                                               // if
                                                                                                               // this
                                                                                                               // is
                                                                                                               // actually
                                                                                                               // brushless
                                                                                                               // later
    private CANSparkMax leftForearmMotor = new CANSparkMax(Constants.Arm.left_forearm_id, MotorType.kBrushless); // Check
                                                                                                                 // if
                                                                                                                 // this
                                                                                                                 // is
                                                                                                                 // actually
                                                                                                                 // brushless
                                                                                                                 // later
    private CANSparkMax rightForearmMotor = new CANSparkMax(Constants.Arm.right_forearm_id, MotorType.kBrushless); // Check
                                                                                                                   // if
                                                                                                                   // this
                                                                                                                   // is
                                                                                                                   // actually
                                                                                                                   // brushless
                                                                                                                   // later

    private MotorControllerGroup upperMotorControllerGroup = new MotorControllerGroup(leftBicepMotor, rightBicepMotor,
            null);
    private MotorControllerGroup lowerMotorControllerGroup = new MotorControllerGroup(leftForearmMotor,
            rightForearmMotor, null);

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
        rightForearmMotor.setInverted(true);
        rightBicepMotor.setInverted(true);

        leftBicepEncoder.setPositionConversionFactor(0);
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
    public void setForearmVoltage(double x, double y) {
        upperMotorControllerGroup.setVoltage(
                forearmController.calculate(leftForearmEncoder.getPosition(), BruteInverseKinematics.calculate(x, y)[0]));
        // taken from wpilib documentation: not too sure how this all works yet
    }

    public void setBicepVoltage(double x, double y) {
        lowerMotorControllerGroup.setVoltage(
                bicepController.calculate(leftBicepEncoder.getPosition(), BruteInverseKinematics.calculate(x, y)[1]));
        // index idk if we want to clean this up lmao
    }

    public void setDegreesUpper(double anlge) {
        leftBicepEncoder.setPosition(anlge);

    }
}