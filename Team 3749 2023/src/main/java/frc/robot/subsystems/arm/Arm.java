package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/**
 * Double jointed arm subsystem built with 2 CANSparkMaxes at each joint and REV
 * Through Bore Encoders
 * 
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 */
public class Arm extends SubsystemBase {

    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);
    private final DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(0);
    private final PIDController shoulderPIDController = new PIDController(Constants.Arm.shoulder_kP, 0, 0);

    private final CANSparkMax elbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(2);
    private final PIDController elbowPIDController = new PIDController(Constants.Arm.elbow_kP, 0, 0);

    private Translation2d position = new Translation2d(0.35, -0.2);

    public Arm() {
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

        shoulderMotor.setIdleMode(IdleMode.kCoast);
        elbowMotor.setIdleMode(IdleMode.kCoast);
    }
    
    public void setArmPosition(Translation2d pos){
        position = pos;
    }

    public Translation2d getArmCoordinate() {
        return ArmKinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle()));
    }

    public void setShoulderVoltage(double voltage) {
        shoulderMotor.setVoltage(voltage);
    }

    public void setElbowVoltage(double voltage) {
        elbowMotor.setVoltage(voltage);
    }

    public double getShoulderAngle() {
        return shoulderAbsoluteEncoder.getDistance();
    }

    public double getElbowAngle() {
        return new Rotation2d(Math.toRadians(elbowAbsoluteEncoder.getAbsolutePosition() * 360 - 180))
                .rotateBy(new Rotation2d(Math.toRadians(180))).getDegrees() - Constants.Arm.elbow_offset;
    }

    public boolean getShoulderAtSetpoint() {
        return shoulderPIDController.atSetpoint();
    }

    public boolean getElbowAtSetpoint() {
        return elbowPIDController.atSetpoint();
    }

    public void stop() {
        elbowMotor.stopMotor();
        shoulderMotor.stopMotor();
    }

    public void stopShoulder() {
        shoulderMotor.stopMotor();
    }

    public void stopElbow() {
        elbowMotor.stopMotor();
    }

    public void periodic() {
        SmartDashboard.putNumber("ARM X CACHE", position.getX());
        SmartDashboard.putNumber("ARM Y CACHE", position.getY());

        SmartDashboard.putNumber("ARM X",
                ArmKinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle())).getX());
        SmartDashboard.putNumber("ARM Y",
                ArmKinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle())).getY());

        SmartDashboard.putNumber("shoulder voltage", shoulderMotor.getBusVoltage() * shoulderMotor.getAppliedOutput());
        SmartDashboard.putNumber("elbow voltage", elbowMotor.getBusVoltage() * elbowMotor.getAppliedOutput());
    }
}
