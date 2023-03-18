// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Limelight;
import org.photonvision.common.hardware.VisionLEDMode;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;

/***
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 * @author Harkirat
 * 
 *         Subsystem class for swerve drive, used to manage four swerve modules
 *         and set their states. Also includes a pose estimator, gyro, and
 *         logging information
 */

public class Swerve extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    // equivilant to a odometer, but also intakes vision
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public Swerve() {
        new Thread(() -> {
            try {
                Thread.sleep(3000);
                gyro.reset();

            } catch (Exception e) {
            }
        }).start();
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics,
                new Rotation2d(0),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(),
                        backLeft.getPosition() },
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0)));

        gyro.calibrate();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getAutoHeading() {
        // return Math.IEEEremainder(gyro.getAngle(), 360);
        return new Rotation2d(Math.toRadians(gyro.getYaw()))
                .rotateBy(new Rotation2d(Math.toRadians(180))).getDegrees();
    }

    public double getHeading() {
        return gyro.getYaw();
    }

    public Rotation2d getAutoRotation2d() {
        return Rotation2d.fromDegrees(-getAutoHeading());

    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-getHeading());
    }

    public Pose2d getPose() {
        Rotation2d rotation = DriverStation.isAutonomous()
                ? getAutoRotation2d()
                : getRotation2d();
        Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();

        return new Pose2d(estimatedPose.getTranslation(), rotation);
    }

    public void resetOdometry(Pose2d pose) {
        Rotation2d rotation = DriverStation.isAutonomous()
                ? getAutoRotation2d()
                : getRotation2d();

        swerveDrivePoseEstimator.resetPosition(rotation,
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(),
                        backLeft.getPosition() },
                pose);
    }

    public void updateOdometry() {
        Rotation2d rotation = DriverStation.isAutonomous()
                ? getAutoRotation2d()
                : getRotation2d();
        swerveDrivePoseEstimator.update(rotation,
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(),
                        backLeft.getPosition() });
        Limelight.updatePoseAprilTags(swerveDrivePoseEstimator);

    }

    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("FL DRIVE ENCODER", frontLeft.getDrivePosition());

        SmartDashboard.putNumber("AUTO Heading", getAutoHeading());

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("pitch", getVerticalTilt());
        SmartDashboard.putNumber("Robot Pose X", getPose().getX());
        SmartDashboard.putNumber("Robot Pose Y", getPose().getY());
        Limelight.logging();
        Limelight.setLED(VisionLEDMode.kOn);
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

    public double getVerticalTilt() {
        return gyro.getPitch();
    }

}
