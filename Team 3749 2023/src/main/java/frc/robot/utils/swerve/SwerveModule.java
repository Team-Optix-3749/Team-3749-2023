


package frc.robot.utils.swerve;

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
 * @author Rohin Sood, Harkirat, Noah Simon
 * @see https://www.youtube.com/watch?v=0Xi9yb1IMyA
 * 
 *     SwerveDrive Module   ( I THINK THIS IS SUPPOSED TO GO IN UTILS)
 */

public class SwerveModule {
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
     * Constructor for Swerve, definegit s our drive and turning motors with encoders as well as the PID
     * 
     * for the parameters, reversed means inverted
     * @param absoluteEncoderOffset The encoder value may be (a consistant amount) higher or lower than the actual rotation of the wheel. This is that measure
     */
    public SwerveModule(SwerveENUMS modulePosition) {
        int driveMotorId;
        int turningMotorId;
        boolean driveMotorReversed;
        boolean turningMotorReversed;
        int absoluteEncoderId;
        double absoluteEncoderOffset;
        boolean absoluteEncoderReversed;
        switch(modulePosition){
            case FRONT_LEFT:
                driveMotorId = Constants.Drivetrain.front_left_drive_id;
                turningMotorId = Constants.Drivetrain.front_left_Turning_id;
                driveMotorReversed = Constants.Drivetrain.front_left_DriveEncoderReversed
                turningMotorReversed = Constants.Drivetrain.front_left_TurningEncoderReversed;
                absoluteEncoderId = Constants.Drivetrain.front_left_driveAbsoluteEncoderPort;
                absoluteEncoderOffset = Constants.Drivetrain.front_left_driveAbsoluteEncoderOffsetRConstants.Drivetrain.ad;
                absoluteEncoderReversed = Constants.Drivetrain.front_left_AbsoluteEncoderReversed;
            case FRONT_RIGHT:
                driveMotorId = Constants.Drivetrain.front_right_drive_id;
                turningMotorId = Constants.Drivetrain.front_right_Turning_id;
                driveMotorReversed = Constants.Drivetrain.front_right_DriveEncoderReversed
                turningMotorReversed = Constants.Drivetrain.front_right_TurningEncoderReversed;
                absoluteEncoderId = Constants.Drivetrain.front_right_driveAbsoluteEncoderPort;
                absoluteEncoderOffset = Constants.Drivetrain.front_right_driveAbsoluteEncoderOffsetRad;
                absoluteEncoderReversed = Constants.Drivetrain.front_right_AbsoluteEncoderReversed;
            case BACK_LEFT:
                driveMotorId = Constants.Drivetrain.back_left_drive_id;
                turningMotorId = Constants.Drivetrain.back_left_Turning_id;
                driveMotorReversed = Constants.Drivetrain.back_left_DriveEncoderReversed
                turningMotorReversed = Constants.Drivetrain.back_left_TurningEncoderReversed;
                absoluteEncoderId = Constants.Drivetrain.back_left_driveAbsoluteEncoderPort;
                absoluteEncoderOffset = Constants.Drivetrain.back_left_driveAbsoluteEncoderOffsetRad;
                absoluteEncoderReversed = Constants.Drivetrain.back_left_AbsoluteEncoderReversed;
            case BACK_RIGHT:
                driveMotorId = Constants.Drivetrain.back_right_drive_id;
                turningMotorId = Constants.Drivetrain.back_right_Turning_id;
                driveMotorReversed = Constants.Drivetrain.back_right_DriveEncoderReversed
                turningMotorReversed = Constants.Drivetrain.back_right_TurningEncoderReversed;
                absoluteEncoderId = Constants.Drivetrain.back_right_driveAbsoluteEncoderPort;
                absoluteEncoderOffset = Constants.Drivetrain.back_right_driveAbsoluteEncoderOffsetRad;
                absoluteEncoderReversed = Constants.Drivetrain.back_right_AbsoluteEncoderReversed;
        }

        // the degrees of the off set value is stored in this code, to be used in a later time when trying to set swerve to align at zero (my grammer good)
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        // gets encoder values in a variable
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        // motor controller groups
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        //inverts something if needed
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        // gets data (from encoder if that is not obvios)
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        
        // conversion factors, ( motor rotation to wheel rotation )
        driveEncoder.setPositionConversionFactor(Constants.SwerveModule.drive_encoder_rotations_to_meter);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveModule.drive_encoder_RPM_to_MPS);
        turningEncoder.setPositionConversionFactor(Constants.SwerveModule.turning_encoder_rotations_to_meter);
        turningEncoder.setVelocityConversionFactor(Constants.SwerveModule.turning_encoder_RPM_to_MPS);

        // creates an object for PID controll (look at top code to know what PID controller does)
        turningPidController = new PIDController(Constants.SwerveModule.turning_p, 0, 0);
        // robot knows that swerve is circle I THINK
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
    }
    // the next few commands are to get position of the motors and the change of speed of the motors in drive or turning
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
        //convert to radians
        angle *= 2.0 * Math.PI;
        // subtracts the offset to get the wheel calibrated
        angle -= absoluteEncoderOffsetRad;
        // multiplies by -1 if motor is inverted
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    // sets relative encoders to the position of the absolute encoder
    public void resetEncoders() {
        //drive is zero while the turrning motor is rotated the amount of degrees it needs(wheel's angle).
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }
    // returns an interatable object that contains the information of swerves present condition
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }


    /*** 
     * completetly defines the state of the swerve drive
     * 
     * @param state the desired state, with a velocity and angle request
     */
    public void setDesiredState(SwerveModuleState state) {
        //prevents the code from going back to zero degrees after joystick is let go (driver convenience)
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        //never move more than 90 degrees per wheel (they will turn the other direction instead)
        state = SwerveModuleState.optimize(state, getState().angle);
        // set motor speed to be the value requested, calculated though constant factors and the current meters/s
        driveMotor.set(state.speedMetersPerSecond / Constants.SwerveModule.turning_encoder_rotations_to_meter);
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