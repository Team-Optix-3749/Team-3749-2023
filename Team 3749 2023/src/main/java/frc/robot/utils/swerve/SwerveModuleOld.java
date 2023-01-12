package frc.robot.utils.swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;

/***
 * @author Rohin Sood
 * @author Harkirat
 * @author Noah Simon
 * @see https://www.youtube.com/watch?v=0Xi9yb1IMyA
 * 
 *      Code to manage each swerve drive module, which contains two motors, two
 *      relative encoders, and an absolute encoder
 */
public class SwerveModuleOld {
    // defined all of the motors along with encoders for those motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    // controll PID, allowing us to get exact motor movement
    private final PIDController turningPidController;

    // This looks at the offset position of the turn motor
    private final AnalogInput absoluteEncoder;
    // if its on an opposing channel the encoder is reversed
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    /***
     * Constructor for Swerve, definegit s our drive and turning motors with
     * encoders as well as the PID
     * 
     * for the parameters, reversed means inverted
     * 
     * @param absoluteEncoderOffset The encoder value may be (a consistant amount)
     *                              higher or lower than the actual rotation of the
     *                              wheel. This is that measure
     */
    public SwerveModuleOld(Constants.SwerveENUMS modulePosition) {
        int drive_motor_id = 0;
        int turning_motor_id = 0;
        boolean drive_motor_reversed = false;
        boolean turning_motor_reversed = false;
        int absolute_encoder_id = 0;
        double absolute_encoder_offset = 0;
        boolean absolute_encoder_reversed = false;

        // Uses enums to set the variables to proper constants. Done here instead of in
        // parameters for organization in the Drivetrain subsystem
        switch (modulePosition) {
            case FRONT_LEFT:
                drive_motor_id = Constants.DrivetrainOld.front_left_drive_id;
                turning_motor_id = Constants.DrivetrainOld.front_left_turning_id;
                drive_motor_reversed = Constants.DrivetrainOld.front_left_drive_encoder_reversed;
                turning_motor_reversed = Constants.DrivetrainOld.front_left_turning_encoder_reversed;
                absolute_encoder_id = Constants.DrivetrainOld.front_left_drive_absolute_encoder_port;
                absolute_encoder_offset = Constants.DrivetrainOld.front_left_drive_absolute_encoder_offset_rad;
                absolute_encoder_reversed = Constants.DrivetrainOld.front_left_drive_absolute_encoder_reversed;
            case FRONT_RIGHT:
                drive_motor_id = Constants.DrivetrainOld.front_right_drive_id;
                turning_motor_id = Constants.DrivetrainOld.front_right_turning_id;
                drive_motor_reversed = Constants.DrivetrainOld.front_right_drive_encoder_reversed;
                turning_motor_reversed = Constants.DrivetrainOld.front_right_turning_encoder_reversed;
                absolute_encoder_id = Constants.DrivetrainOld.front_right_drive_absolute_encoder_port;
                absolute_encoder_offset = Constants.DrivetrainOld.front_right_drive_absolute_encoder_offset_rad;
                absolute_encoder_reversed = Constants.DrivetrainOld.front_right_drive_absolute_encoder_reversed;
            case BACK_LEFT:
                drive_motor_id = Constants.DrivetrainOld.back_left_drive_id;
                turning_motor_id = Constants.DrivetrainOld.back_left_turning_id;
                drive_motor_reversed = Constants.DrivetrainOld.back_left_drive_encoder_reversed;
                turning_motor_reversed = Constants.DrivetrainOld.back_left_turning_encoder_reversed;
                absolute_encoder_id = Constants.DrivetrainOld.back_left_drive_absolute_encoder_port;
                absolute_encoder_offset = Constants.DrivetrainOld.back_left_drive_absolute_encoder_offset_rad;
                absolute_encoder_reversed = Constants.DrivetrainOld.back_left_drive_absolute_encoder_reversed;
            case BACK_RIGHT:
                drive_motor_id = Constants.DrivetrainOld.back_right_drive_id;
                turning_motor_id = Constants.DrivetrainOld.back_right_turning_id;
                drive_motor_reversed = Constants.DrivetrainOld.back_right_drive_encoder_reversed;
                turning_motor_reversed = Constants.DrivetrainOld.back_right_turning_encoder_reversed;
                absolute_encoder_id = Constants.DrivetrainOld.back_right_drive_absolute_encoder_port;
                absolute_encoder_offset = Constants.DrivetrainOld.back_right_drive_absolute_encoder_offset_rad;
                absolute_encoder_reversed = Constants.DrivetrainOld.back_right_drive_absolute_encoder_reversed;
        }

        this.absoluteEncoderOffsetRad = absolute_encoder_offset;
        this.absoluteEncoderReversed = absolute_encoder_reversed;

        absoluteEncoder = new AnalogInput(absolute_encoder_id);

        driveMotor = new CANSparkMax(drive_motor_id, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turning_motor_id, MotorType.kBrushless);

        driveMotor.setInverted(drive_motor_reversed);
        turningMotor.setInverted(turning_motor_reversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // conversion factors, ( motor rotation to wheel rotation )
        driveEncoder.setPositionConversionFactor(Constants.SwerveModuleOld.drive_encoder_rotations_to_meter);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveModuleOld.drive_encoder_RPM_to_MPS);
        turningEncoder.setPositionConversionFactor(Constants.SwerveModuleOld.turning_encoder_rotations_to_meter);
        turningEncoder.setVelocityConversionFactor(Constants.SwerveModuleOld.turning_encoder_RPM_to_MPS);

        turningPidController = new PIDController(Constants.SwerveModuleOld.turning_p.get(), 0, 0);
        // The PID will understand that it is working in a circle and will loop around
        // after pi or -pi
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    // Get the absolute encoder value in radians
    public double getAbsoluteEncoderRad() {

        // Gets percentage of rotation read by encoder
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        // convert to radians
        angle *= 2.0 * Math.PI;
        // subtracts the offset to get the wheel calibrated
        angle -= absoluteEncoderOffsetRad;
        // multiplies by -1 if motor is inverted
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    // sets relative encoders to the position of the absolute encoder
    public void resetEncoders() {
        // drive is zero while the turrning motor is rotated the amount of degrees it
        // needs (wheel's angle).
        driveEncoder.setPosition(0);
        // turningEncoder.setPosition(getAbsoluteEncoderRad());
        turningEncoder.setPosition(0);

    }

    // returns an interatable object that contains the information of swerves
    // present condition
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    // /**
    // * Returns the current position of the module.
    // *
    // * @return The current position of the module.
    // */
    public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
    driveEncoder.getPosition(), new
    Rotation2d(turningEncoder.getPosition()));
    }

    /***
     * completetly defines the state of the swerve drive
     * 
     * @param state the desired state, with a velocity and angle request
     */
    public void setDesiredState(SwerveModuleState state) {
        // prevents the code from going back to zero degrees after joystick is let go
        // (driver convenience)
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // never move more than 90 degrees per wheel (they will turn the other direction
        // instead)
        state = SwerveModuleState.optimize(state, getState().angle);
        // set motor speed to be the value requested, calculated though constant factors
        // and the current meters/s
        driveMotor.set(state.speedMetersPerSecond / Constants.SwerveModuleOld.turning_encoder_rotations_to_meter);
        // pid to calculate turning position,
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        // SmartDashboard logging
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}