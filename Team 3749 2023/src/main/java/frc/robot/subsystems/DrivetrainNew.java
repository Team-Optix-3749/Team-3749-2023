package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.swerve.SwerveModuleNew;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainNew extends SubsystemBase {

    private final SwerveModuleNew frontLeft = new SwerveModuleNew(Constants.SwerveENUMS.FRONT_LEFT);
    private final SwerveModuleNew frontRight = new SwerveModuleNew(Constants.SwerveENUMS.FRONT_RIGHT);
    private final SwerveModuleNew backLeft = new SwerveModuleNew(Constants.SwerveENUMS.BACK_LEFT);
    private final SwerveModuleNew backRight = new SwerveModuleNew(Constants.SwerveENUMS.BACK_RIGHT);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            Constants.DrivetrainNew.kinematics,
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

    @Override
    public void periodic() {
        
        // Update the odometry in the periodic block
        odometry.update(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
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

        var swerveModuleStates = Constants.DrivetrainNew.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.DrivetrainNew.max_speed);

        for (int i = 0; i < swerveModuleStates.length; i++) {
            System.out.println(swerveModuleStates[i]);
        }
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
        SmartDashboard.putNumber("drive feed forward 0",frontLeft.setDesiredState(swerveModuleStates[0])[0]);
        SmartDashboard.putNumber("drive feed forward 1",frontLeft.setDesiredState(swerveModuleStates[1])[0]);
        SmartDashboard.putNumber("drive feed forward 2",frontLeft.setDesiredState(swerveModuleStates[2])[0]);
        SmartDashboard.putNumber("drive feed forward 3",frontLeft.setDesiredState(swerveModuleStates[3])[0]);
        SmartDashboard.putNumber("drive output 0",frontLeft.setDesiredState(swerveModuleStates[0])[1]);
        SmartDashboard.putNumber("drive output 1",frontLeft.setDesiredState(swerveModuleStates[1])[1]);
        SmartDashboard.putNumber("drive output 2",frontLeft.setDesiredState(swerveModuleStates[2])[1]);
        SmartDashboard.putNumber("drive output 3",frontLeft.setDesiredState(swerveModuleStates[3])[1]);
        SmartDashboard.putNumber("turn feed forward 0",frontLeft.setDesiredState(swerveModuleStates[0])[2]);
        SmartDashboard.putNumber("turn feed forward 1",frontLeft.setDesiredState(swerveModuleStates[1])[2]);
        SmartDashboard.putNumber("turn feed forward 2",frontLeft.setDesiredState(swerveModuleStates[2])[2]);
        SmartDashboard.putNumber("turn feed forward 3",frontLeft.setDesiredState(swerveModuleStates[3])[2]);
        SmartDashboard.putNumber("turn output 0",frontLeft.setDesiredState(swerveModuleStates[0])[3]);
        SmartDashboard.putNumber("turn output 1",frontLeft.setDesiredState(swerveModuleStates[1])[3]);
        SmartDashboard.putNumber("turn output 2",frontLeft.setDesiredState(swerveModuleStates[2])[3]);
        SmartDashboard.putNumber("turn output 3",frontLeft.setDesiredState(swerveModuleStates[3])[3]);




    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, Constants.DrivetrainNew.max_speed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (Constants.DrivetrainNew.gyro_reversed ? -1.0 : 1.0);
    }

}
