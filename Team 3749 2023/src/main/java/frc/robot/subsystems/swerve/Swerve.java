// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.sql.Driver;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
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
    private SwerveModule[] modules = new SwerveModule[4];

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    // equivilant to a odometer, but also intakes vision
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private final PIDController turnController = new PIDController(0.0335, 0.00, 0);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    private ShuffleData<Double> robotHeading = new ShuffleData<Double>("Swerve", "Robot Heading", 0.0);
    private ShuffleData<Double> pitch = new ShuffleData<Double>("Swerve", "Robot Pitch", 0.0);
    private ShuffleData<Double> robotPoseX = new ShuffleData<Double>("Swerve", "Robot Pose X", 0.0);
    private ShuffleData<Double> robotPoseY = new ShuffleData<Double>("Swerve", "Robot Pose Y", 0.0);

    private boolean flipGyro = true;

    public Swerve() {
        new Thread(() -> {
            try {
                Thread.sleep(3000);
                gyro.reset();

            } catch (Exception e) {
            }
        }).start();

        for (int i = 0; i < 4; i++){
            modules[i] = new SwerveModule(DriveConstants.diveMotorPorts[i], 
            DriveConstants.turningMotorPorts[i], 
            DriveConstants.driveAbsoluteEncoderReversed[i], 
            DriveConstants.turningEncoderReversed[i],
            DriveConstants.absoluteEncoderPorts[i],
            DriveConstants.driveAbsoluteEncoderOffsetDeg[i],
            DriveConstants.driveAbsoluteEncoderReversed[i]);
            
        }

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics,
                new Rotation2d(0),
                new SwerveModulePosition[] { modules[0].getPosition(),  modules[1].getPosition(),  modules[2].getPosition(),
                    modules[3].getPosition() },
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

    public void setFlipGyro(boolean bool) {
        flipGyro = bool;
    }

    public void resetGyro() {
        gyro.reset();
        System.out.println("RESET");
    }

    public double getAutoHeading() {

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
        Rotation2d rotation = DriverStation.getAlliance() == Alliance.Blue
                ? (flipGyro ? getAutoRotation2d() : getRotation2d())
                : getRotation2d();
        Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();

        return new Pose2d(estimatedPose.getTranslation(), rotation);
    }

    public boolean getFlipGyro() {
        return flipGyro;
    }

    public void resetOdometry(Pose2d pose) {
        Rotation2d rotation = DriverStation.getAlliance() == Alliance.Blue
                ? (flipGyro ? getAutoRotation2d() : getRotation2d())
                : getRotation2d();

        swerveDrivePoseEstimator.resetPosition(rotation,
                new SwerveModulePosition[] {  modules[0].getPosition(),  modules[1].getPosition(),  modules[2].getPosition(),
                    modules[3].getPosition() },
                pose);
    }

    public void updateOdometry() {
        Rotation2d rotation = DriverStation.getAlliance() == Alliance.Blue
                ? (flipGyro ? getAutoRotation2d() : getRotation2d())
                : getRotation2d();
        swerveDrivePoseEstimator.update(rotation,
                new SwerveModulePosition[] {  modules[0].getPosition(),  modules[1].getPosition(),  modules[2].getPosition(),
                    modules[3].getPosition() });

    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return swerveDrivePoseEstimator;
    }

    public void stopModules() {
        for (SwerveModule module : modules){
            module.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (flipGyro && !DriverStation.isAutonomous()) {
            for (int i = 0; i < 4; i++) {
                desiredStates[i].speedMetersPerSecond = -desiredStates[i].speedMetersPerSecond;

            }
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        for (int i = 0; i<4; i++){
            modules[i].setDesiredState(desiredStates[i]);
        }



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

    public void toggleSpeed() {

        if (Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond == 5) {
            Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond = 3;
            Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond = 3;
        } else {
            Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond = 5;
            Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond = 5;
        }

    }

    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumberArray("Odometry",
        new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });
        double[] realStates = { 
            modules[0].getAbsoluteEncoderRad(),
            modules[0].getDriveVelocity(),
            modules[1].getAbsoluteEncoderRad(),
            modules[1].getDriveVelocity(),
            modules[2].getAbsoluteEncoderRad(),
            modules[2].getDriveVelocity(),
            modules[3].getAbsoluteEncoderRad(),
            modules[3].getDriveVelocity()};
        
        double[] theoreticalStates = {
                modules[0].getTheoreticalState().angle.getDegrees(),
                modules[0].getTheoreticalState().speedMetersPerSecond,
                modules[1].getTheoreticalState().angle.getDegrees(),
                modules[1].getTheoreticalState().speedMetersPerSecond,
                modules[2].getTheoreticalState().angle.getDegrees(),
                modules[2].getTheoreticalState().speedMetersPerSecond,
                modules[3].getTheoreticalState().angle.getDegrees(),
                modules[3].getTheoreticalState().speedMetersPerSecond,
            };
            SmartDashboard.putNumberArray("Theoretical States", theoreticalStates);

        SmartDashboard.putNumberArray("Real Staets", realStates);
        robotHeading.set(getHeading());
        pitch.set(getVerticalTilt());
        robotPoseX.set(getPose().getX());
        robotPoseY.set(getPose().getY());
    }
}