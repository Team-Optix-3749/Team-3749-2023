package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.Constants;

public class ArmIOSparkMax implements ArmIO {
    

    private final CANSparkMax elbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(2);

    // Shoulder motor
    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);
    private final DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(0);

    public ArmIOSparkMax() {

    }

    /**
     * Set shoulder motor voltage
     * 
     * @param voltage
     */
    public void setShoulderVoltage(double voltage) {
        shoulderMotor.setVoltage(voltage);
    }

    /**
     * Set elbow motor voltage
     * 
     * @param voltage
     */
    public void setElbowVoltage(double voltage) {
        elbowMotor.setVoltage(voltage);
    }

    /**
     * Get current shoulder angle
     * 
     * @return shoulder angle as double
     */
    public double getShoulderAngle() {
        return shoulderAbsoluteEncoder.getDistance();
    }

    /**
     * Get current elbow angle
     * 
     * @return elbow angle as double
     */
    public double getElbowAngle() {
        return new Rotation2d(Math.toRadians(elbowAbsoluteEncoder.getAbsolutePosition() * 360 - 180))
                .rotateBy(new Rotation2d(Math.toRadians(180))).getDegrees() - Constants.Arm.elbow_offset;
    }
}
