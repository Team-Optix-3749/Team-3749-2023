package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

/**
 * Double jointed arm subsystem built with 2 CANSparkMaxes at each joint and REV
 * Through Bore Encoders
 * 
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 */

public class ArmSparkMax implements ArmIO {

    // Shoulder motor
    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);
    private final DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(9);
    private final PIDController shoulderPIDController = new PIDController(Constants.Arm.shoulder_kP, 0, 0);

    // Elbow motor
    private final CANSparkMax elbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(8);
    private final PIDController elbowPIDController = new PIDController(Constants.Arm.elbow_kP, 0, 0);


    public ArmSparkMax() {
        shoulderMotor.restoreFactoryDefaults();
        elbowMotor.restoreFactoryDefaults();

        // elbow offset is done in the get angle method
        shoulderAbsoluteEncoder.setPositionOffset(Constants.Arm.shoulder_offset);
        shoulderAbsoluteEncoder.setDistancePerRotation(360);
        elbowAbsoluteEncoder.setDistancePerRotation(360);

        elbowMotor.setInverted(true);
        shoulderMotor.setInverted(false);

        shoulderPIDController.setTolerance(0);
        elbowPIDController.setTolerance(0);

        shoulderMotor.setSmartCurrentLimit(35, 60);
        elbowMotor.setSmartCurrentLimit(35, 60);


    }

    /**
     * Set shoulder motor voltage. The voltage will not be set if kill is true
     * 
     * @param voltage
     */
    private void setShoulderVoltage(double voltage) {
        shoulderMotor.setVoltage(voltage);
    }

    /**
     * Set elbow motor voltage. The voltage will not be set if kill is true
     * 
     * @param voltage
     */
    private void setElbowVoltage(double voltage) {
        elbowMotor.setVoltage(voltage);
    }

    /**
     * Get current shoulder angle
     * 
     * @return shoulder angle as double
     */
    private double getShoulderAngle() {
        return (shoulderAbsoluteEncoder.getAbsolutePosition() - Constants.Arm.shoulder_offset) * 360;
        // return (shoulderAbsoluteEncoder.getAbsolutePosition() * 360.0 );
    }

    /**
     * Get current elbow angle
     * 
     * @return elbow angle as double
     */
    private double getElbowAngle() {
        return new Rotation2d(Math.toRadians(elbowAbsoluteEncoder.getAbsolutePosition() * 360 - 180))
                .rotateBy(new Rotation2d(Math.toRadians(180))).getDegrees() - Constants.Arm.elbow_offset;
        // return new
        // Rotation2d(Math.toRadians(elbowAbsoluteEncoder.getAbsolutePosition() * 360 -
        // 180))
        // .rotateBy(new Rotation2d(Math.toRadians(180))).getDegrees();
    }

    /**
     * Move arm to set position using PID and DJ FF
     * 
     * @throws Exception
     */
    @Override
    public void moveArm(Translation2d setpoint, ArmDynamics dynamics) throws Exception {
        double shoulderAngle = ArmKinematics.inverse(setpoint.getX(), setpoint.getY()).getFirst();
        double elbowAngle = ArmKinematics.inverse(setpoint.getX(), setpoint.getY()).getSecond();

        double[] feedForwardOutput = dynamics.feedforward(VecBuilder.fill(shoulderAngle, elbowAngle)).getData();

        setShoulderVoltage(shoulderPIDController.calculate(getShoulderAngle(), shoulderAngle) + feedForwardOutput[0]);
        setElbowVoltage(elbowPIDController.calculate(getElbowAngle(), elbowAngle) + feedForwardOutput[1]);

        // shoulderPID.set(shoulderPIDController.calculate(getShoulderAngle(), shoulderAngle));
        // shoulderFF.set(feedForwardOutput[0]);

        // elbowPID.set(elbowPIDController.calculate(getElbowAngle(), elbowAngle));
        // elbowFF.set(feedForwardOutput[1]);
    }

    /**
     * Get current arm pose as Translation2d
     * 
     * @return arm coordianates as Translation2d
     */

    private Translation2d getArmCoordinate() {
        return ArmKinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle()));
    }



    // /**
    //  * Use this method to test & log the FF output for a given XY
    //  * 
    //  * @param x
    //  * @param y
    //  * @throws Exception Terminates the process in the event the XY exceeds the arm
    //  *                   radius
    //  */
    // private void ffTesting(double x, double y) throws Exception {
    //     double shoulderAngle = ArmKinematics.inverse(x, y).getFirst();
    //     double elbowAngle = ArmKinematics.inverse(x, y).getSecond();

    //     double[] feedForwardOutput = dynamics.feedforward(VecBuilder.fill(shoulderAngle, elbowAngle)).getData();

    //     shoulderFF.set(feedForwardOutput[0]);
    //     elbowFF.set(feedForwardOutput[1]);
    // }

    /**
     * @param mode IdleMode of the arm
     */
    @Override
    public void setArmBrakeMode(boolean enable) {
        if (enable) {
            shoulderMotor.setIdleMode(IdleMode.kBrake);
            elbowMotor.setIdleMode(IdleMode.kBrake);
        } else {
            shoulderMotor.setIdleMode(IdleMode.kCoast);
            elbowMotor.setIdleMode(IdleMode.kCoast);
        }

    }

    @Override
    public void updateData(ArmData data) {

        data.shoulderAngle = getShoulderAngle();
        data.elbowAngle = getElbowAngle();
        data.shoulderLigament.setAngle(getShoulderAngle());
        data.elbowLigament.setAngle(getElbowAngle());
        data.position = getArmCoordinate();
        data.shoulderVoltage = shoulderMotor.getBusVoltage() * shoulderMotor.getAppliedOutput();
        data.elbowVoltage = elbowMotor.getBusVoltage() * elbowMotor.getAppliedOutput();
    }
}