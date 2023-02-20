package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;

/**
 * Double jointed arm subsystem built with 2 CANSparkMaxes at each joint and REV
 * Through Bore Encoders
 * 
 * TODO: increase tolerance on sting, change to brake when at setpoint
 * 
 * @author Rohin Sood
 */
public class ArmReal extends Arm {

    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id,
            MotorType.kBrushless);
    private final DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(0);
    private final PIDController shoulderPIDController = new PIDController(Constants.Arm.shoulderKP.get(), 0, 0);

    private final CANSparkMax elbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(1);
    private final PIDController elbowPIDController = new PIDController(Constants.Arm.elbowKP.get(), 0, 0);

    private final SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

    public ArmReal() {
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

    @Override
    public void setShoulder(double percent) {
        boolean past_min_limit = getShoulderAngle() <= Constants.Arm.shoulder_min_angle && percent > 0;
        boolean past_max_limit = getShoulderAngle() >= Constants.Arm.shoulder_max_angle && percent < 0;
        if (past_min_limit || past_max_limit) {
            return;
        }
        shoulderMotor.set(percent);
    }

    @Override
    public void setElbow(double percent) {
        elbowMotor.set(percent);
    }

    @Override
    public double getShoulderAngle() {
        return shoulderAbsoluteEncoder.getDistance();
    }

    @Override
    public double getElbowAngle() {
        return elbowAbsoluteEncoder.getDistance();
    }

    @Override
    public void setShoulderAngle(double angle) {
        shoulderPIDController.setSetpoint(angle);

        shoulderMotor.set(
                shoulderPIDController.calculate(
                        shoulderAbsoluteEncoder.getDistance()));
    }

    @Override
    public void setElbowAngle(double angle) {
        elbowPIDController.setSetpoint(angle);

        elbowMotor.set(
                elbowPIDController.calculate(
                        elbowAbsoluteEncoder.getDistance()));
    }

    @Override
    public boolean getShoulderAtSetpoint() {
        return shoulderPIDController.atSetpoint();
    }

    @Override
    public boolean getElbowAtSetpoint() {
        return elbowPIDController.atSetpoint();
    }

    @Override
    public void setArmAngle(double shoulder_angle, double elbow_angle) {
        setShoulderAngle(shoulder_angle);
        setElbowAngle(elbow_angle);
    }

    @Override
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

    @Override
    public void stop() {
        elbowMotor.stopMotor();
        shoulderMotor.stopMotor();
    }

    @Override
    public void stopShoulder() {
        shoulderMotor.stopMotor();
    }

    @Override
    public void stopElbow() {
        elbowMotor.stopMotor();
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        shoulderMotor.setIdleMode(idleMode);
        elbowMotor.setIdleMode(idleMode);
    }

    @Override
    public void setArmTolerance(double tolerance) {
        shoulderPIDController.setTolerance(tolerance);
        elbowPIDController.setTolerance(tolerance);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Right Amps", shoulderMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elbow right Amps", elbowMotor.getOutputCurrent());

        SmartDashboard.putNumber("shoulder angle", shoulderAbsoluteEncoder.getDistance());
        SmartDashboard.putNumber("elbow angle", elbowAbsoluteEncoder.getDistance());

        elbowPIDController.setP(Constants.Arm.elbowKP.get());
        shoulderPIDController.setP(Constants.Arm.shoulderKP.get());
    }

}
