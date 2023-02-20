package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/**
 * Double jointed arm subsystem built with 2 CANSparkMaxes at each joint and REV
 * Through Bore Encoders
 * 
 * TODO: increase tolerance on sting, change to brake when at setpoint
 * 
 * @author Rohin Sood
 */
public class Arm extends SubsystemBase {

    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id,
            MotorType.kBrushless);
    private final DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(0);
    private final PIDController shoulderPIDController = new PIDController(Constants.Arm.shoulderKP.get(), 0, 0);

    private final CANSparkMax elbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(1);
    private final PIDController elbowPIDController = new PIDController(Constants.Arm.elbowKP.get(), 0, 0);

    private final SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

    public Arm() {
        shoulderMotor.restoreFactoryDefaults();
        elbowMotor.restoreFactoryDefaults();

        shoulderAbsoluteEncoder.setPositionOffset(Constants.Arm.shoulder_offset);
        elbowAbsoluteEncoder.setPositionOffset(Constants.Arm.elbow_offset);
        shoulderAbsoluteEncoder.setDistancePerRotation(360);
        elbowAbsoluteEncoder.setDistancePerRotation(360);

        elbowMotor.setInverted(true);
        shoulderMotor.setInverted(false);

        presetChooser.setDefaultOption("Stowed", 0);
        presetChooser.addOption("DS", 1);
        SmartDashboard.putData(presetChooser);

        shoulderPIDController.setTolerance(3);
        elbowPIDController.setTolerance(5);

        setIdleMode(IdleMode.kCoast);
    }

    public void setShoulder(double percent) {
        boolean past_min_limit = getShoulderAngle() <= Constants.Arm.shoulder_min_angle && percent > 0;
        boolean past_max_limit = getShoulderAngle() >= Constants.Arm.shoulder_max_angle && percent < 0;
        if (past_min_limit || past_max_limit) {
            return;
        }
        shoulderMotor.set(percent);
    }

    public void setElbow(double percent) {
        elbowMotor.set(percent);
    }

    public double getShoulderAngle() {
        return shoulderAbsoluteEncoder.getDistance();
    }

    public double getElbowAngle() {
        return elbowAbsoluteEncoder.getDistance();
    }

    public void setShoulderAngle(double angle) {
        shoulderPIDController.setSetpoint(angle);

        shoulderMotor.set(
                shoulderPIDController.calculate(
                        shoulderAbsoluteEncoder.getDistance()));
    }

    public void setElbowAngle(double angle) {
        elbowPIDController.setSetpoint(angle);

        elbowMotor.set(
                elbowPIDController.calculate(
                        elbowAbsoluteEncoder.getDistance()));
    }

    public boolean getShoulderAtSetpoint() {
        return shoulderPIDController.atSetpoint();
    }

    public boolean getElbowAtSetpoint() {
        return elbowPIDController.atSetpoint();
    }

    public void setArmAngle(double shoulder_angle, double elbow_angle) {
        setShoulderAngle(shoulder_angle);
        setElbowAngle(elbow_angle);
    }

    public void setArmPreset() {
        double shoulder_angle;
        double elbow_angle;

        switch (presetChooser.getSelected()) {
            case (0):
                shoulder_angle = Constants.Arm.ShoulderSetpoints.STOWED.angle;
                elbow_angle = Constants.Arm.ElbowSetpoints.STOWED.angle;
                break;
            case (1):
                shoulder_angle = Constants.Arm.ShoulderSetpoints.DOUBLE_SUBSTATION.angle;
                elbow_angle = Constants.Arm.ElbowSetpoints.DOUBLE_SUBSTATION.angle;
                break;
            default:
                shoulder_angle = Constants.Arm.ShoulderSetpoints.STOWED.angle;
                elbow_angle = Constants.Arm.ElbowSetpoints.STOWED.angle;
                break;
        }

        setShoulderAngle(shoulder_angle);
        setElbowAngle(elbow_angle);
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

    public void setIdleMode(IdleMode idleMode) {
        shoulderMotor.setIdleMode(idleMode);
        elbowMotor.setIdleMode(idleMode);
    }

    public void setArmTolerance(double tolerance) {
        shoulderPIDController.setTolerance(tolerance);
        elbowPIDController.setTolerance(tolerance);
    }

    public void periodic() {
        SmartDashboard.putNumber("Shoulder Right Amps", shoulderMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elbow right Amps", elbowMotor.getOutputCurrent());

        SmartDashboard.putNumber("shoulder angle", shoulderAbsoluteEncoder.getDistance());
        SmartDashboard.putNumber("elbow angle", elbowAbsoluteEncoder.getDistance());

        elbowPIDController.setP(Constants.Arm.elbowKP.get());
        shoulderPIDController.setP(Constants.Arm.shoulderKP.get());
    }

}
