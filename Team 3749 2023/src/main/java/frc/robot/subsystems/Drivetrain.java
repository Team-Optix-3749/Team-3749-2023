package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.swerve.SwerveENUMS;
import frc.robot.utils.swerve.SwerveModule;


/***
 * @author Rohin Sood
 * @author Harkirat Httar
 * @author Noah Simon
 * @see https://www.youtube.com/watch?v=0Xi9yb1IMyA
 * 
 *     Subsystem for swerve drive
 */
public class Drivetrain extends SubsystemBase {

    
    private final SwerveModule front_left_ = new SwerveModule(SwerveENUMS.FRONT_LEFT);

    private final SwerveModule frontRight = new SwerveModule(SwerveENUMS.FRONT_RIGHT);

    private final SwerveModule back_left_ = new SwerveModule(SwerveENUMS.BACK_LEFT);

    private final SwerveModule backRight = new SwerveModule(SwerveENUMS.BACK_RIGHT);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final edu.wpi.first.math.kinematics.SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics,
            new Rotation2d(0));

    public Drivetrain() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), front_left_.getState(), frontRight.getState(), back_left_.getState(),backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        front_left_.stop();
        frontRight.stop();
        back_left_.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        front_left_.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        back_left_.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
