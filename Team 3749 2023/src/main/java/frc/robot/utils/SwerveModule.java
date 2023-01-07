package frc.robot.utils;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;

/***
 * @author Rohin Sood
 * @see https://www.youtube.com/watch?v=0Xi9yb1IMyA
 * 
 *      Class for a swerve module
 */
public class SwerveModule {

    private final WPI_TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absolute_encoder_reversed;
    private final double absolute_encoder_offset;

    // initializes the swerve module with the given parameters
    public SwerveModule(int drive_id, int turning_id, boolean drive_reverse, boolean turning_reversed,
            int absolute_encoder_Id, double absolute_encoder_offset, boolean absolute_encoder_reversed) {

        this.absolute_encoder_offset = absolute_encoder_offset;
        this.absolute_encoder_reversed = absolute_encoder_reversed;
        absoluteEncoder = new AnalogInput(absolute_encoder_Id);

        driveMotor = new WPI_TalonFX(drive_id);
        turningMotor = new CANSparkMax(turning_id, MotorType.kBrushless);

        driveMotor.setInverted(drive_reverse);
        turningMotor.setInverted(turning_reversed);

        turningEncoder = turningMotor.getEncoder();

        turningEncoder.setPositionConversionFactor(Constants.SwerveModule.turning_encoder_rotations_to_meter);
        turningEncoder.setVelocityConversionFactor(Constants.SwerveModule.turning_encoder_RPM_to_MPS);

        turningPidController = new PIDController(Constants.SwerveModule.turning_p, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    /**
     * Gets the drive position in meters
     */
    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition() * Constants.SwerveModule.drive_encoder_rotations_to_meter;
    }

    /**
     * Gets the turning position in meters
     */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /**
     * Gets the drive velocity in meters/second
     */
    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * Constants.SwerveModule.drive_encoder_RPM_to_MPS ;
    }

    /**
     * Gets the drive velocity in meters/second
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absolute_encoder_offset;
        return angle * (absolute_encoder_reversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.Drivetrain.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
