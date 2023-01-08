package frc.robot.subsystems;

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

public class Swervedrive {
    // defined all of the motors along with encoders for those motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    // controll PID, exact rotations of a motor, correct me if I am wrong Noah
    private final PIDController turningPidController;

    // This looks at the offset position of the turn motor
    private final AnalogInput absoluteEncoder;
    // if its on an opposing channel the encoder is reversed
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    //  Takes in the id's and positions along with other information(reversed(i think this means inverted)), There is also the offset value stored in a double. 
    public Swervedrive(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        // the degrees of the off set value is stored in this code, to be used in a later time when trying to set swerve to alighn at zero (my grammer good)
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        // gets encoder values in a variable
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        // motor controller groups
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        //inverts somehing if needed
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        // gets data (from encoder if that is not obvios)
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        
        // conversion factors, ( motor rotation to wheel rotation )
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        // creats an object for PID controll (look at top code to know what PID controller does)
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // robot knows that swerve is circle I THINK
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
    }
    // the next few commands ore to get position of the motors and the change of speed of the motors in drive or turning
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
    // The absolute encoder value as well
    public double getAbsoluteEncoderRad() {
        // Gets percentage of rotation
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        //convert to radians
        angle *= 2.0 * Math.PI;
        // subtracts the offset to get the wheel calibrated
        angle -= absoluteEncoderOffsetRad;
        // multiplies by -1 if motor is inverted
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
    // gives values from absolute encoders wich always know there location
    public void resetEncoders() {
        //drive is zero while the turrning motor is rotated the amount of degrees it needs(wheel's angle).
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }
    //Data for WPI lib
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        //prevents the code from going back to zero degrees after joystick is let go (driver convenience)
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        //never move more than 90 degrees per wheel
        state = SwerveModuleState.optimize(state, getState().angle);
        // scale down velocity
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // pid to calculate turning position
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        // error code for help if something fails
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}