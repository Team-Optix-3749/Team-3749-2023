package frc.robot.testing;

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
import frc.robot.utils.Constants.SwerveENUMS;
import frc.robot.utils.swerve.SwerveModuleNew;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainTesting extends SubsystemBase {

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

    public DrivetrainTesting() {
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
    public void moveIndividualModule(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Constants.SwerveENUMS modulePosition) {

        var swerveModuleStates = Constants.DrivetrainNew.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.DrivetrainNew.max_speed);
        double[] state = new double[6];
        if (modulePosition == SwerveENUMS.FRONT_LEFT){
            state = frontLeft.setDesiredState(swerveModuleStates[1]);
            logModuleState(state);
        }
        
        else if (modulePosition == SwerveENUMS.FRONT_RIGHT){
            state = frontRight.setDesiredState(swerveModuleStates[0]);
            logModuleState(state);
        }

        else if (modulePosition == SwerveENUMS.BACK_LEFT){
            state = backLeft.setDesiredState(swerveModuleStates[3]);
            logModuleState(state);

        }
        else if (modulePosition == SwerveENUMS.BACK_RIGHT){
            state = backRight.setDesiredState(swerveModuleStates[2]);
            logModuleState(state);
        }
        
    }

    public void logModuleState(double[] state) {
        // Smart dashboard logging
        String[] valueNames = {" drive feed forward", " drive output", " turn feed forward", " turn output", "state meters per second", "state radians"};
        for (int valIndex = 0; valIndex <6; valIndex++){
            SmartDashboard.putNumber(valueNames[valIndex], state[valIndex]);
        }
        SmartDashboard.putNumber("YAW",gyro.getYaw());
        SmartDashboard.putNumber("PITCH",gyro.getPitch());
        SmartDashboard.putNumber("ROLL",gyro.getRoll());
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
