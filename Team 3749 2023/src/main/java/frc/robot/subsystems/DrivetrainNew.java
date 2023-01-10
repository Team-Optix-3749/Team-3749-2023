package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.utils.swerve.SwerveModuleNew;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainNew extends SubsystemBase {
    // Locations for the swerve drive modules relative to the robot center.
    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveModuleNew frontLeft = new SwerveModuleNew(Constants.SwerveENUMS.FRONT_LEFT);
    private final SwerveModuleNew frontRight = new SwerveModuleNew(Constants.SwerveENUMS.FRONT_RIGHT);
    private final SwerveModuleNew backLeft = new SwerveModuleNew(Constants.SwerveENUMS.BACK_LEFT);
    private final SwerveModuleNew backRight = new SwerveModuleNew(Constants.SwerveENUMS.BACK_RIGHT);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Creating my kinematics object using the module locations
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            kinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });

    public DrivetrainNew() {
        gyro.reset();
    }

    /** These two methods added by me, not in the docs */
    public double getHeading() {
        // loops around 360 degrees
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var SwerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(SwerveModuleStates, Constants.DrivetrainNew.max_speed);
        frontLeft.setDesiredState(SwerveModuleStates[0]);
        frontRight.setDesiredState(SwerveModuleStates[1]);
        backLeft.setDesiredState(SwerveModuleStates[2]);
        backRight.setDesiredState(SwerveModuleStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        odometry.update(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    /***
     * added by me
     * @param desiredStates a set of 4 swerve module states that will all be normalized and set to the proper modules
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Normalize speeds so that two motors at different speeds, but both greater than max speed, will run at proportionate speeds 

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DrivetrainOld.physical_max_speed_meters_per_second);
        // SwerveModuleKinematics.normalizeWheelSpeeds(desiredStates, Constants.Drivetrain.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** Method made by me, not in the docs */
    @Override
    public void periodic() {
        updateOdometry();
    }
}
