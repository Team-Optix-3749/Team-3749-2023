package frc.robot.testing;

import java.util.function.Supplier;

import com.ctre.phoenixpro.hardware.CANcoder;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;

/***
 * @author Rohin Sood
 * 
 *         Code to manage each swerve drive module, which contains two motors,
 *         two relative encoders, and an absolute encoder
 */

public class SwerveModuleTesting {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final CANcoder turningEncoder;

    // The absolute encoder do not have a regular get position, but seem to give
    // back these suppliers
    Supplier<Double> turningPositionSupplier;
    Supplier<Double> turningVelocitySupplier;

    private final PIDController drivePIDController = new PIDController(Constants.DrivetrainNew.driveKP.get(),
            Constants.DrivetrainNew.driveKI.get(), Constants.DrivetrainNew.driveKD.get());

    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
            Constants.DrivetrainNew.turningKP.get(),
            Constants.DrivetrainNew.turningKI.get(),
            Constants.DrivetrainNew.turningKD.get(),
            new TrapezoidProfile.Constraints(
                    Constants.SwerveModuleNew.max_angular_velocity,
                    Constants.SwerveModuleNew.max_angular_acceleration));

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            Constants.DrivetrainNew.driveKS.get(),
            Constants.DrivetrainNew.driveKV.get());
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(
            Constants.DrivetrainNew.turningKS.get(),
            Constants.DrivetrainNew.turningKV.get());

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     */
    public SwerveModuleTesting(Constants.SwerveENUMS modulePosition) {

        int drive_motor_id = 0;
        int turning_motor_id = 0;
        int absolute_encoder_port = 0;
        boolean drive_motor_reversed = false;
        boolean turning_motor_reversed = false;

        // Uses enums to set the variables to proper constants. Done here instead of in
        // parameters for organization in the Drivetrain subsystem
        if (modulePosition == Constants.SwerveENUMS.FRONT_LEFT) {
            drive_motor_id = Constants.SwerveModuleNew.front_left_drive_id;
            turning_motor_id = Constants.SwerveModuleNew.front_left_turning_id;
            absolute_encoder_port = Constants.SwerveModuleNew.front_left_absolute_encoder_port;
            drive_motor_reversed = Constants.SwerveModuleNew.front_left_drive_encoder_reversed;
            turning_motor_reversed = Constants.SwerveModuleNew.front_left_turning_encoder_reversed;
        } else if (modulePosition == Constants.SwerveENUMS.FRONT_RIGHT) {
            drive_motor_id = Constants.SwerveModuleNew.front_right_drive_id;
            turning_motor_id = Constants.SwerveModuleNew.front_right_turning_id;
            absolute_encoder_port = Constants.SwerveModuleNew.front_right_absolute_encoder_port;
            drive_motor_reversed = Constants.SwerveModuleNew.front_right_drive_encoder_reversed;
            turning_motor_reversed = Constants.SwerveModuleNew.front_right_turning_encoder_reversed;
        } else if (modulePosition == Constants.SwerveENUMS.BACK_LEFT) {
            drive_motor_id = Constants.SwerveModuleNew.back_left_drive_id;
            turning_motor_id = Constants.SwerveModuleNew.back_left_turning_id;
            absolute_encoder_port = Constants.SwerveModuleNew.back_left_absolute_encoder_port;

            drive_motor_reversed = Constants.SwerveModuleNew.back_left_drive_encoder_reversed;
            turning_motor_reversed = Constants.SwerveModuleNew.back_left_turning_encoder_reversed;
        } else if (modulePosition == Constants.SwerveENUMS.BACK_RIGHT) {
            drive_motor_id = Constants.SwerveModuleNew.back_right_drive_id;
            turning_motor_id = Constants.SwerveModuleNew.back_right_turning_id;
            absolute_encoder_port = Constants.SwerveModuleNew.back_right_absolute_encoder_port;
            drive_motor_reversed = Constants.SwerveModuleNew.back_right_drive_encoder_reversed;
            turning_motor_reversed = Constants.SwerveModuleNew.back_right_turning_encoder_reversed;
        }

        driveMotor = new CANSparkMax(drive_motor_id, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turning_motor_id, MotorType.kBrushless);

        driveMotor.setInverted(drive_motor_reversed);
        turningMotor.setInverted(turning_motor_reversed);

        // Drive motor is relative, turning is absolute
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = new CANcoder(absolute_encoder_port);

        turningPositionSupplier = turningEncoder.getPosition().asSupplier();
        turningVelocitySupplier = turningEncoder.getVelocity().asSupplier();
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
                driveEncoder.getVelocity(), new Rotation2d(turningPositionSupplier.get()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(), new Rotation2d(turningPositionSupplier.get()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     * @return returns a double[] of the different outputs and feedforwards. Drive,
     *         then turn. Feedforward, then PID
     */
    public double[] setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(turningPositionSupplier.get()));
        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        final double driveFeedforward = this.driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(turningPositionSupplier.get(),
                state.angle.getRadians());

        final double turnFeedforward = this.turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        // We add feed forward and PID. PID handles correcting where we are, Feedforward
        // handles where we are going, adding them sets it up for the best of both
        setVoltage(driveOutput + driveFeedforward, turnOutput + turnFeedforward);
        // returns our output data, in case we want it
        return new double[] { driveFeedforward, driveOutput, turnFeedforward, turnOutput,
                state.speedMetersPerSecond, state.angle.getRadians() };
    }

    public void setVoltage(double drive_voltage, double turn_voltage) {
        driveMotor.setVoltage(drive_voltage);
        turningMotor.setVoltage(turn_voltage);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     * @return returns a double[] of the different outputs and feedforwards. Drive,
     *         then turn. Feedforward, then PID
     */
    public double[] setDesiredDrive(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(turningPositionSupplier.get()));
        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        final double driveFeedforward = this.driveFeedforward.calculate(state.speedMetersPerSecond);

        // We add feed forward and PID. PID handles correcting where we are, Feedforward
        // handles where we are going, adding them sets it up for the best of both
        setVoltage(driveOutput + driveFeedforward, 0);
        // returns our output data, in case we want it
        return new double[] { driveFeedforward, driveOutput, 0, 0,
                state.speedMetersPerSecond, state.angle.getRadians() };
    }

    public double[] setDesiredTurning(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(turningPositionSupplier.get()));

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(turningPositionSupplier.get(),
                state.angle.getRadians());

        final double turnFeedforward = this.turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        // We add feed forward and PID. PID handles correcting where we are, Feedforward
        // handles where we are going, adding them sets it up for the best of both
        setVoltage(0, turnOutput + turnFeedforward);
        // returns our output data, in case we want it
        return new double[] { 0, 0, turnFeedforward, turnOutput,
                state.speedMetersPerSecond, state.angle.getRadians() };
    }

    public void resetEncoders() {
        // drive is zero while the turrning motor is rotated the amount of degrees it
        // needs (wheel's angle).
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
    }
}
