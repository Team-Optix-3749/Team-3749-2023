// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
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

    private static SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private final PIDController turnController = new PIDController(0.045, 0.00, 0);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    private ShuffleData<Double> robotHeading = new ShuffleData<Double>("Swerve", "Robot Heading", 0.0);
    private ShuffleData<Double> pitch = new ShuffleData<Double>("Swerve", "Robot Pitch", 0.0);
    private ShuffleData<Double> robotPoseX = new ShuffleData<Double>("Swerve", "Robot Pose X", 0.0);
    private ShuffleData<Double> robotPoseY = new ShuffleData<Double>("Swerve", "Robot Pose Y", 0.0);

    public Swerve() {
        new Thread(() -> {
            try {
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

        // swerveDrivePoseEstimator.setVisionMeasurementStdDevs(null);
        gyro.calibrate();
        turnController.enableContinuousInput(-180, 180);
    }

    public void drive(double xSpeed, double ySpeed, double thetaSpeed) {

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, xSpeed, 0, getRotation2d());
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);

        setModuleStates(moduleStates);
    }

    public void stop() {

        drive(0, 0, 0);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-getHeading());
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
        // update pose estimation using encoders and gyro
        swerveDrivePoseEstimator.update(getRotation2d(),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(),
                        backLeft.getPosition() });

    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return swerveDrivePoseEstimator;
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
        // negative so that we move towards the target, not away
        double turning_speed = -turnController.calculate(getHeading(), angle);
        turning_speed = turningLimiter.calculate(turning_speed);
        // signs the speed so we move in the correct direction
        // turning_speed = Math.abs(turning_speed) * Math.signum(getHeading());

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
        return gyro.getPitch();
    }

    public PIDController getTurnController() {
        return turnController;
    }

    public SlewRateLimiter getTurnLimiter() {
        return turningLimiter;
    }

    @Override
    public void periodic() {
        updateOdometry();

        robotHeading.set(getHeading());
        pitch.set(getVerticalTilt());
        robotPoseX.set(getPose().getX());
        robotPoseY.set(getPose().getY());
    }
}
