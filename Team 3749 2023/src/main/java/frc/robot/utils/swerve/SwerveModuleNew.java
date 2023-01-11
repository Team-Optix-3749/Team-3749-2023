package frc.robot.utils.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.Constants;

/***
 * @author Rohin Sood
 * 
 *         Code to manage each swerve drive module, which contains two motors,
 *         two relative encoders, and an absolute encoder
 */

public class SwerveModuleNew {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    Constants.SwerveModuleNew.max_angular_velocity,
                    Constants.SwerveModuleNew.max_angular_acceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, Constants.DrivetrainNew.driveKV.get());
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, Constants.DrivetrainNew.turningKV.get());

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     */
    public SwerveModuleNew(Constants.SwerveENUMS modulePosition) {

        int drive_motor_id = 0;
        int turning_motor_id = 0;
        boolean drive_motor_reversed = false;
        boolean turning_motor_reversed = false;

        // Uses enums to set the variables to proper constants. Done here instead of in
        // parameters for organization in the Drivetrain subsystem
        switch (modulePosition) {
            case FRONT_LEFT:
                drive_motor_id = Constants.DrivetrainOld.front_left_drive_id;
                turning_motor_id = Constants.DrivetrainOld.front_left_turning_id;
                drive_motor_reversed = Constants.DrivetrainOld.front_left_drive_encoder_reversed;
                turning_motor_reversed = Constants.DrivetrainOld.front_left_turning_encoder_reversed;
            case FRONT_RIGHT:
                drive_motor_id = Constants.DrivetrainOld.front_right_drive_id;
                turning_motor_id = Constants.DrivetrainOld.front_right_turning_id;
                drive_motor_reversed = Constants.DrivetrainOld.front_right_drive_encoder_reversed;
                turning_motor_reversed = Constants.DrivetrainOld.front_right_turning_encoder_reversed;
            case BACK_LEFT:
                drive_motor_id = Constants.DrivetrainOld.back_left_drive_id;
                turning_motor_id = Constants.DrivetrainOld.back_left_turning_id;
                drive_motor_reversed = Constants.DrivetrainOld.back_left_drive_encoder_reversed;
                turning_motor_reversed = Constants.DrivetrainOld.back_left_turning_encoder_reversed;
            case BACK_RIGHT:
                drive_motor_id = Constants.DrivetrainOld.back_right_drive_id;
                turning_motor_id = Constants.DrivetrainOld.back_right_turning_id;
                drive_motor_reversed = Constants.DrivetrainOld.back_right_drive_encoder_reversed;
                turning_motor_reversed = Constants.DrivetrainOld.back_right_turning_encoder_reversed;
        }

        driveMotor = new CANSparkMax(drive_motor_id, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turning_motor_id, MotorType.kBrushless);

        driveMotor.setInverted(drive_motor_reversed);
        turningMotor.setInverted(turning_motor_reversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        driveEncoder.setPositionConversionFactor(Constants.SwerveModuleNew.drive_encoder_rotations_to_meter);
        turningEncoder.setVelocityConversionFactor(Constants.SwerveModuleNew.drive_encoder_RPM_to_MPS);

        // Set the distance (in this case, angle) in radians per pulse for the turning
        // encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        turningEncoder.setPositionConversionFactor(Constants.SwerveModuleNew.turning_encoder_rotations_to_meter);
        turningEncoder.setVelocityConversionFactor(Constants.SwerveModuleNew.turning_encoder_RPM_to_MPS);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(turningEncoder.getPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        final double driveFeedforward = this.driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(turningEncoder.getPosition(),
                state.angle.getRadians());

        final double turnFeedforward = this.turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput + driveFeedforward);
        turningMotor.setVoltage(turnOutput + turnFeedforward);
    }

    public void resetEncoders() {
        // drive is zero while the turrning motor is rotated the amount of degrees it
        // needs (wheel's angle).
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
    }
}
