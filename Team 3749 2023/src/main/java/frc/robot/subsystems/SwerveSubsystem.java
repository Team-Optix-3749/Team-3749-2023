// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.utils.AprilTagGetters;

public class SwerveSubsystem extends SubsystemBase {
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


    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(3000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        AprilTagGetters.setSwervePoseEstimator(new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(),
            backLeft.getPosition() });
        gyro.calibrate();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        // return Math.IEEEremainder(gyro.getAngle(), 360);
        return gyro.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-getHeading());
    }

    public Pose2d getPose() {
        return AprilTagGetters.getSwerveDrivePoseEstimator().getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        AprilTagGetters.getSwerveDrivePoseEstimator().resetPosition(getRotation2d(),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(),
                        backLeft.getPosition() },
                pose);
    }

    @Override
    public void periodic() {
        AprilTagGetters.getSwerveDrivePoseEstimator().update(getRotation2d(),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(),
                        backLeft.getPosition() });
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Pose X", getPose().getX());
        SmartDashboard.putNumber("Robot Pose Y", getPose().getY());


        SmartDashboard.putNumber("frontLeft encoder", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("frontRight encoder", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("backLeft encoder", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("backRight encoder", backRight.getAbsoluteEncoderRad());

        SmartDashboard.putNumber("frontLeft turning pos", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("frontRight turning pos", frontRight.getTurningPosition());
        SmartDashboard.putNumber("backLeft turning pos", backLeft.getTurningPosition());
        SmartDashboard.putNumber("backRight turning pos", backRight.getTurningPosition());
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
