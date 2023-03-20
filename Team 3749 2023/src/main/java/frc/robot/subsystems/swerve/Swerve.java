// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIONavX2;
import frc.robot.subsystems.swerve.modules.SwerveModuleIO;
import frc.robot.subsystems.swerve.modules.SwerveModuleIOInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.SwerveModuleIOSparkMax;
import frc.robot.subsystems.swerve.modules.SwerveModuleIO.SwerveModuleIOInputs;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;

/***
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan ____
 * @author Harkirat ____
 * 
 *         Subsystem class for swerve drive, used to manage four swerve modules
 *         and set their states. Also includes a pose estimator, gyro, and
 *         logging information
 */

public class Swerve extends SubsystemBase {
    private final SwerveModuleIO frontLeft = new SwerveModuleIOSparkMax(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    SwerveModuleIOInputsAutoLogged frontLeftInputs = new SwerveModuleIOInputsAutoLogged();

    private final SwerveModuleIO frontRight = new SwerveModuleIOSparkMax(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
    SwerveModuleIOInputsAutoLogged frontRightInputs = new SwerveModuleIOInputsAutoLogged();

    private final SwerveModuleIO backLeft = new SwerveModuleIOSparkMax(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
    SwerveModuleIOInputsAutoLogged backLeftInputs = new SwerveModuleIOInputsAutoLogged();

    private final SwerveModuleIO backRight = new SwerveModuleIOSparkMax(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    SwerveModuleIOInputsAutoLogged backRightInputs = new SwerveModuleIOInputsAutoLogged();

    // equivilant to a odometer, but also intakes vision
    private static SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private final PIDController turnController = new PIDController(0.045, 0.00, 0);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    private final GyroIO gyroIO;

    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    public Swerve() {
        gyroIO = new GyroIONavX2();
        new Thread(() -> {
            try {
                gyroIO.calibrate();
                Thread.sleep(3000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics,
                new Rotation2d(0),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(),
                        backLeft.getPosition() },
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0)));

        turnController.enableContinuousInput(-180, 180);
    }

    public void zeroHeading() {
        gyroIO.reset();
    }

    public double getHeading() {
        // return Math.IEEEremainder(gyro.getAngle(), 360);
        return gyroInputs.yawAngle;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();
        return new Pose2d(estimatedPose.getTranslation(), getRotation2d());
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrivePoseEstimator.resetPosition(getRotation2d(),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(),
                        backLeft.getPosition() },
                pose);
    }

    public void updateOdometry() {
        swerveDrivePoseEstimator.update(getRotation2d(),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(),
                        backLeft.getPosition() });
    }

    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("pitch", getVerticalTilt());
        SmartDashboard.putNumber("Robot Pose X", getPose().getX());
        SmartDashboard.putNumber("Robot Pose Y", getPose().getY());
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
        frontLeft.updateInputs(frontLeftInputs);
        frontRight.updateInputs(frontRightInputs);
        backLeft.updateInputs(backLeftInputs);
        backRight.updateInputs(backRightInputs);
        Logger.getInstance().processInputs("Drive/FrontLeft", frontLeftInputs);
        Logger.getInstance().processInputs("Drive/FrontRight", frontRightInputs);
        Logger.getInstance().processInputs("Drive/BackLeft", backLeftInputs);
        Logger.getInstance().processInputs("Drive/BackRight", backRightInputs);
        double[] states = new double [] {
            frontLeftInputs.turnPositionRad, frontLeftInputs.driveVelocityMetersPerSec,
            frontRightInputs.turnPositionRad, frontRightInputs.driveVelocityMetersPerSec,
            backLeftInputs.turnPositionRad, backLeftInputs.driveVelocityMetersPerSec,
            backRightInputs.turnPositionRad, backRightInputs.driveVelocityMetersPerSec
        };

        Logger.getInstance().recordOutput("Drive/Modules", states);


        // for (int i = 0; i < 4; i++) {
        // moduleIOs[i].updateInputs(moduleInputs[i]);
        // Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i),
        // moduleInputs[i]);
        // }

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    /***
     * 
     * @param angle the angle to move at, in degrees, -180 to 180
     * @param speed the speed to move at, 0-1
     */
    public void moveAtAngle(double angle, double speed) {
        angle = Math.toRadians(angle);
        double xSpeed = Math.cos(angle) * speed;
        double ySpeed = -Math.sin(angle) * speed;
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, 0, getRotation2d());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        setModuleStates(moduleStates);
    }

    /***
     * 
     * @param angle the rotational angle to move to, -180 to 180
     */
    public void turnToRotation(double angle) {
        SmartDashboard.putNumber("ANGLE SETPOINT", angle);

        // negative so that we move towards the target, not away
        double turning_speed = -turnController.calculate(getHeading(), angle);
        turning_speed = turningLimiter.calculate(turning_speed);
        // signs the speed so we move in the correct direction
        // turning_speed = Math.abs(turning_speed) * Math.signum(getHeading());

        SmartDashboard.putNumber("SPEEEEED", turning_speed);
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 0, turning_speed, getRotation2d());
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);
        // 6. Output each module states to wheels
        setModuleStates(moduleStates);
    }

    public double getVerticalTilt() {
        return gyroInputs.pitchAngle;
    }

    public PIDController getTurnController() {
        return turnController;
    }

    public SlewRateLimiter getTurnLimiter() {
        return turningLimiter;
    }
}
