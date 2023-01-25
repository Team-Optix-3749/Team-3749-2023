package frc.robot.testing;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.SwerveENUMS;

/***
 * @author Noah Simon
 * @summary
 *          the Test Drivetrain subsystem
 */
public class DrivetrainTesting extends SubsystemBase {

    private final SendableChooser<Constants.SwerveENUMS> moduleChooser = new SendableChooser<>();
    private final SendableChooser<Constants.DriveTypeTestingENUMS> driveTypeChooser = new SendableChooser<>();
    private Constants.SwerveENUMS selected_module;
    private Constants.DriveTypeTestingENUMS selected_drive_type;

    private final SwerveModuleTesting frontLeft = new SwerveModuleTesting(Constants.SwerveENUMS.FRONT_LEFT);
    private final SwerveModuleTesting frontRight = new SwerveModuleTesting(Constants.SwerveENUMS.FRONT_RIGHT);
    private final SwerveModuleTesting backLeft = new SwerveModuleTesting(Constants.SwerveENUMS.BACK_LEFT);
    private final SwerveModuleTesting backRight = new SwerveModuleTesting(Constants.SwerveENUMS.BACK_RIGHT);

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
        moduleChooser.setDefaultOption("Front Left", Constants.SwerveENUMS.FRONT_LEFT);
        moduleChooser.addOption("Front Right", Constants.SwerveENUMS.FRONT_RIGHT);
        moduleChooser.addOption("Back Left", Constants.SwerveENUMS.BACK_LEFT);
        moduleChooser.addOption("Back Right", Constants.SwerveENUMS.BACK_RIGHT);
        SmartDashboard.putData("Test Module Chooser", moduleChooser);
        driveTypeChooser.setDefaultOption("Drive and Turning", Constants.DriveTypeTestingENUMS.DRIVE_AND_TURNING);
        driveTypeChooser.addOption("Drive", Constants.DriveTypeTestingENUMS.DRIVE);
        driveTypeChooser.addOption("Turning", Constants.DriveTypeTestingENUMS.TURNING);
        SmartDashboard.putData("Test Drive Type Chooser", driveTypeChooser);
        gyro.reset();
    }

    @Override
    public void periodic() {
        logIndividualModuleEncoderValues();

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
     *                      field_
     */
    public void moveIndividualModule(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        selected_module = moduleChooser.getSelected();
        selected_drive_type = driveTypeChooser.getSelected();
        var swerveModuleStates = Constants.DrivetrainNew.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.DrivetrainNew.max_speed);

        double[] state = new double[6];
        if (selected_module == SwerveENUMS.FRONT_LEFT) {
            if (selected_drive_type == Constants.DriveTypeTestingENUMS.DRIVE_AND_TURNING) {
                state = frontLeft.setDesiredState(swerveModuleStates[1]);

            } else if (selected_drive_type == Constants.DriveTypeTestingENUMS.DRIVE) {
                state = frontLeft.setDesiredDrive(swerveModuleStates[1]);
            } else if (selected_drive_type == Constants.DriveTypeTestingENUMS.TURNING) {
                state = frontLeft.setDesiredTurning(swerveModuleStates[1]);
            }
        }

        else if (selected_module == SwerveENUMS.FRONT_RIGHT) {
            if (selected_drive_type == Constants.DriveTypeTestingENUMS.DRIVE_AND_TURNING) {
                state = frontRight.setDesiredState(swerveModuleStates[0]);
            } else if (selected_drive_type == Constants.DriveTypeTestingENUMS.DRIVE) {
                state = frontRight.setDesiredDrive(swerveModuleStates[0]);
            } else if (selected_drive_type == Constants.DriveTypeTestingENUMS.TURNING) {
                state = frontRight.setDesiredTurning(swerveModuleStates[0]);
            }

        }

        else if (selected_module == SwerveENUMS.BACK_LEFT) {
            if (selected_drive_type == Constants.DriveTypeTestingENUMS.DRIVE_AND_TURNING) {
                state = backLeft.setDesiredState(swerveModuleStates[3]);
            } else if (selected_drive_type == Constants.DriveTypeTestingENUMS.DRIVE) {
                state = backLeft.setDesiredDrive(swerveModuleStates[3]);
            } else if (selected_drive_type == Constants.DriveTypeTestingENUMS.TURNING) {
                state = backLeft.setDesiredTurning(swerveModuleStates[3]);
            }

        } else if (selected_module == SwerveENUMS.BACK_RIGHT) {
            if (selected_drive_type == Constants.DriveTypeTestingENUMS.DRIVE_AND_TURNING) {
                state = backRight.setDesiredState(swerveModuleStates[2]);
            } else if (selected_drive_type == Constants.DriveTypeTestingENUMS.DRIVE) {
                state = backRight.setDesiredDrive(swerveModuleStates[2]);
            } else if (selected_drive_type == Constants.DriveTypeTestingENUMS.TURNING) {
                state = backRight.setDesiredTurning(swerveModuleStates[2]);
            }

        }
        logModuleState(state);

    }

    public void logModuleState(double[] state) {
        // Smart dashboard logging
        String[] valueNames = { " drive feed forward", " drive output", " turn feed forward", " turn output",
                "state meters per second", "state radians" };
        for (int valIndex = 0; valIndex < 6; valIndex++) {
            SmartDashboard.putNumber(valueNames[valIndex], state[valIndex]);
        }

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

    public void toggleIdleMode() {
        frontLeft.toggleIdleMode();
        frontRight.toggleIdleMode();
        backLeft.toggleIdleMode();
        backRight.toggleIdleMode();
    }

    public void logIndividualModuleEncoderValues() {
        if (moduleChooser.getSelected() == SwerveENUMS.FRONT_LEFT) {
            SmartDashboard.putNumber("Absolute Encoder", frontLeft.getAbsoluteEncoderValue());
            SmartDashboard.putNumber("Drive Encoder", frontLeft.getDriveEncoderValue());
        } else if (moduleChooser.getSelected() == SwerveENUMS.FRONT_RIGHT) {
            SmartDashboard.putNumber("Absolute Encoder", frontRight.getAbsoluteEncoderValue());
            SmartDashboard.putNumber("Drive Encoder", frontRight.getDriveEncoderValue());
        } else if (moduleChooser.getSelected() == SwerveENUMS.BACK_LEFT) {
            SmartDashboard.putNumber("Absolute Encoder", backLeft.getAbsoluteEncoderValue());
            SmartDashboard.putNumber("Drive Encoder", backLeft.getDriveEncoderValue());
        } else if (moduleChooser.getSelected() == SwerveENUMS.BACK_RIGHT) {
            SmartDashboard.putNumber("Absolute Encoder", backRight.getAbsoluteEncoderValue());
            SmartDashboard.putNumber("Drive Encoder", backRight.getDriveEncoderValue());
        }
    }

    public void logAbsoluteEncoderValues() {
        SmartDashboard.putNumber("Front Left Absolute Encoder", frontLeft.getAbsoluteEncoderValue());
        SmartDashboard.putNumber("Front Right Absolute Encoder", frontRight.getAbsoluteEncoderValue());
        SmartDashboard.putNumber("Back Left Absolute Encoder", backLeft.getAbsoluteEncoderValue());
        SmartDashboard.putNumber("Back Right Absolute Encoder", backRight.getAbsoluteEncoderValue());

    }

    public void logDriveEncoderValues() {
        SmartDashboard.putNumber("Front Left Drive Encoder", frontLeft.getDriveEncoderValue());
        SmartDashboard.putNumber("Front Right Drive Encoder", frontRight.getDriveEncoderValue());
        SmartDashboard.putNumber("Back Left Drive Encoder", backLeft.getDriveEncoderValue());
        SmartDashboard.putNumber("Back Right Drive Encoder", backRight.getDriveEncoderValue());
    }

}
