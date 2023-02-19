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
 * @author Rohin Sood
 */
public class ArmReal extends Arm {

    private final CANSparkMax leftShoulderMotor = new CANSparkMax(Constants.Arm.left_shoulder_id, MotorType.kBrushless);
    private final CANSparkMax rightShoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id,
            MotorType.kBrushless);
    private final DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(0);
    private final PIDController shoulderPIDController = new PIDController(Constants.Arm.shoulderKP.get(), 0, 0);

    private final CANSparkMax leftElbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private final CANSparkMax rightElbowMotor = new CANSparkMax(Constants.Arm.right_elbow_id, MotorType.kBrushless);
    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(1);
    private final PIDController elbowPIDController = new PIDController(Constants.Arm.elbowKP.get(), 0, 0);

    private final SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

    public ArmReal() {
        leftShoulderMotor.restoreFactoryDefaults();
        rightShoulderMotor.restoreFactoryDefaults();
        leftElbowMotor.restoreFactoryDefaults();
        rightElbowMotor.restoreFactoryDefaults();

        shoulderAbsoluteEncoder.setPositionOffset(Constants.Arm.shoulder_offset);
        elbowAbsoluteEncoder.setPositionOffset(Constants.Arm.elbow_offset);
        shoulderAbsoluteEncoder.setDistancePerRotation(360);
        elbowAbsoluteEncoder.setDistancePerRotation(360);

        leftShoulderMotor.setInverted(true);
        leftElbowMotor.setInverted(true);

        shoulderPIDController.setTolerance(3);
        elbowPIDController.setTolerance(3);

        presetChooser.setDefaultOption("Stowed", 0);
        presetChooser.addOption("DS", 1);
        SmartDashboard.putData(presetChooser);
    }

    @Override
    public void setShoulder(double percent) {
        boolean past_min_limit = getShoulderAngle() <= Constants.Arm.shoulder_min_angle && percent < 0;
        boolean past_max_limit = getShoulderAngle() >= Constants.Arm.shoulder_max_angle && percent > 0;
        if (past_min_limit || past_max_limit) {
            return;
        }
        leftShoulderMotor.set(percent);
        // rightShoulderMotor.set(percent);
    }

    @Override
    public void setElbow(double percent) {
        leftElbowMotor.set(percent);
        // rightElbowMotor.set(percent);
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

        leftShoulderMotor.set(
                shoulderPIDController.calculate(
                        shoulderAbsoluteEncoder.getDistance()));

        rightShoulderMotor.set(
                shoulderPIDController.calculate(
                        shoulderAbsoluteEncoder.getDistance()));
    }

    @Override
    public void setElbowAngle(double angle) {
        elbowPIDController.setSetpoint(angle);

        leftElbowMotor.set(
                elbowPIDController.calculate(
                        elbowAbsoluteEncoder.getDistance()));

        rightElbowMotor.set(
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
        rightElbowMotor.stopMotor();
        rightShoulderMotor.stopMotor();
        leftElbowMotor.stopMotor();
        leftShoulderMotor.stopMotor();
    }

    @Override
    public void stopShoulder() {
        setIdleMode(IdleMode.kCoast);
        rightShoulderMotor.stopMotor();
        leftShoulderMotor.stopMotor();
    }

    @Override
    public void stopElbow() {
        setIdleMode(IdleMode.kCoast);
        rightElbowMotor.stopMotor();
        leftElbowMotor.stopMotor();
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        leftShoulderMotor.setIdleMode(idleMode);
        rightShoulderMotor.setIdleMode(idleMode);
        leftElbowMotor.setIdleMode(idleMode);
        rightElbowMotor.setIdleMode(idleMode);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Left Amps", leftShoulderMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shoulder Right Amps", rightShoulderMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elbow Left Amps", leftElbowMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elbow right Amps", rightElbowMotor.getOutputCurrent());

        SmartDashboard.putNumber("shoulder angle", shoulderAbsoluteEncoder.getDistance());
        SmartDashboard.putNumber("elbow angle", elbowAbsoluteEncoder.getDistance());

        elbowPIDController.setP(Constants.Arm.elbowKP.get());
        shoulderPIDController.setP(Constants.Arm.shoulderKP.get());

        SmartDashboard.putString("idlemode", leftShoulderMotor.getIdleMode() == IdleMode.kBrake ? "brake" : "coast");

        SmartDashboard.putBoolean("SHOULDER AT SETPOINT", getShoulderAtSetpoint());
        SmartDashboard.putBoolean("ELBOW AT SETPOINT", getElbowAtSetpoint());

        SmartDashboard.putNumber("S ERROR", getShoulderAngle() - shoulderPIDController.getSetpoint());
        SmartDashboard.putNumber("E ERROR", getElbowAngle() - elbowPIDController.getSetpoint());
    }

}
