package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

/***
 * @author Noah Simon
 * @author Rohin Sood
 * @author Harkirat
 * @summary
 *          Controlling the Drivetrain subsystem through use of joysticks, drive
 *          motors only
 */
public class DrivetrainNew extends SubsystemBase {

    private final SwerveModuleNew frontLeft = new SwerveModuleNew(Constants.SwerveENUMS.FRONT_LEFT);
    private final SwerveModuleNew frontRight = new SwerveModuleNew(Constants.SwerveENUMS.FRONT_RIGHT);
    private final SwerveModuleNew backLeft = new SwerveModuleNew(Constants.SwerveENUMS.BACK_LEFT);
    private final SwerveModuleNew backRight = new SwerveModuleNew(Constants.SwerveENUMS.BACK_RIGHT);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            Constants.DrivetrainNew.kinematics,
            getRotation2d(),
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
                getRotation2d(),
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
                getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    public Rotation2d getRotation2d() {	
        return gyro.getRotation2d();	
    }

    public double getHeading() {	
        // loops around 360 degrees	
        return Math.IEEEremainder(gyro.getAngle(), 360);	
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
        // converts speeds to chassis speeds and then chassis speeds to module states
        var swerveModuleStates = Constants.DrivetrainNew.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        // makes all module speeds proportional to the max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.DrivetrainNew.max_speed);
        // states array is just for logging. set desired state is what's really doing
        // the job here
        double[][] states = new double[4][4];
        states[0] = frontRight.setDesiredState(swerveModuleStates[0]);
        states[1] = frontLeft.setDesiredState(swerveModuleStates[1]);
        states[2] = backRight.setDesiredState(swerveModuleStates[2]);
        states[3] = backLeft.setDesiredState(swerveModuleStates[3]);

        logModuleStates(states);
    }

    public void turnToZeroHeading() {
        double heading = getHeading();
        double speed = heading / 360; // PID Would be better, but this works for now.

        // Set chassis speed to be only forward, relative to the field.
        ChassisSpeeds chassisSpeeds;
        // SPEED DEF NEEDS TO BE DIFFERENT
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, speed, getRotation2d());
        SwerveModuleState[] states = Constants.DrivetrainNew.kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public double getVerticalTilt() {
        return gyro.getPitch();
    }

    /***
     * 
     * @param desiredStates a set of 4 swerve module states that will all be
     *                      normalized and set to the proper modules
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Normalize speeds so that two motors at different speeds, but both greater
        // than max speed, will run at proportionate speeds

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                Constants.DrivetrainNew.max_speed);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void logModuleStates(double[][] states) {
        // Smart dashboard logging
        String[] moduleNames = { "FR", "FL", "BR", "BL" };
        String[] valueNames = { " drive feed forward", " drive output", " turn feed forward", " turn output",
                "state meters per second", "state radians" };
        for (int modIndex = 0; modIndex < 4; modIndex++) {
            for (int valIndex = 0; valIndex < 6; valIndex++) {

                SmartDashboard.putNumber(valueNames[valIndex] + moduleNames[modIndex], states[modIndex][valIndex]);
            }
        }
        SmartDashboard.putNumber("YAW", gyro.getYaw());
        SmartDashboard.putNumber("PITCH", gyro.getPitch());
        SmartDashboard.putNumber("ROLL", gyro.getRoll());
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
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (Constants.DrivetrainNew.gyro_reversed ? -1.0 : 1.0);
    }

    public void toggleIdleMode() {
        frontLeft.toggleIdleMode();
        frontRight.toggleIdleMode();
        backLeft.toggleIdleMode();
        backRight.toggleIdleMode();

    }

}
