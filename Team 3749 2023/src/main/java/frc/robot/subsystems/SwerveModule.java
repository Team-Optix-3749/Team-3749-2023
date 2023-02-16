package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/***
 * @author Noah Simon
 * @author Raadwan Masum
 * @author Rohin Sood
 * @author Harkirat ____
 * 
 *         Object to manage each individual swerve module, including a drive
 *         motor, a turning motor, a drive encoder, and an Absolute CanCoder
 * 
 */
public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
            boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {

        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        absoluteEncoder.configMagnetOffset(absoluteEncoderOffset);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningMotor.setIdleMode(IdleMode.kCoast);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    /**
     * @return module drive position
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * @return module turning position
     */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /**
     * @return module drive velocity
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * @return module turning velocity
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * @return module absolute encoder value in radians
     */
    public double getAbsoluteEncoderRad() {
        return ((absoluteEncoder.getAbsolutePosition() / 180 * Math.PI))
                * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * Reset module encoders
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * @return module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    /**
     * @return module position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    /**
     * Set module state
     * @param state
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        double drive_speed = state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

        double turning_speed = turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());

        driveMotor.set(drive_speed);
        turningMotor.set(turning_speed);
    }

    /**
     * Stop swerve module
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    /**
     * Turn module to degree
     * @param angleDegrees
     */
    public void turnToDegrees(double angleDegrees) {
        double angleRad = Units.degreesToRadians(angleDegrees);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), angleRad));

    }
}