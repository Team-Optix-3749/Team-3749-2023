// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/***
 * @author Noah Simon
 * @author Raadwan ___
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

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

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

        SmartDashboard.putNumber(String.valueOf(absoluteEncoderId), absoluteEncoder.getAbsolutePosition());

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

    public double getAbsoluteEncoderRad() {
        // double angle = absoluteEncoder.getBusVoltage() /
        // RobotController.getVoltage5V();
        // angle *= 2.0 * Math.PI;
        // angle -= absoluteEncoderOffsetRad;
        // return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        SmartDashboard.putNumber("ABS POS " + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition());

        return ((absoluteEncoder.getAbsolutePosition() / 180 * Math.PI)) * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad()));
    }

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
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] State", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    // Turn to the set degree amount, -180 to 180
    public void turnToDegrees(double angleDegrees) {
        double angleRad = Units.degreesToRadians(angleDegrees);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), angleRad));

    }

}