package frc.robot.commands.swerve;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Noah Simon
 * 
 *         Moves the robot a specific amount forwrad on a button press. For
 *         testing purposes
 */
public class MoveDistance extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    Swerve swerve;

    double setpoint;
    double dist_traveled = 0;
    double start_point;

    PIDController driveController = new PIDController(1.6, 0, 0.01);
    PIDController turnController = new PIDController(0.005, 0.001, 0);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

            
    /***
     * 
     * @param swerve the subsystem
     * @param dist            how far to to move in meters
     */
    public MoveDistance(Swerve swerve, double dist) {
        this.swerve = swerve;
        this.setpoint = dist;
        // start_point
        addRequirements(swerve);
    }

    // Run on command init
    @Override
    public void initialize() {
        System.out.println("Initalize BABBYYYYYYYYYYYYYY hlleo");
        swerve.stopModules();
        start_point = swerve.getPose().getX();
    }

    // Run every 20 ms
    @Override
    public void execute() {
        double heading = swerve.getRotation2d().getDegrees();
        // How inaccurate we are willing to be in reference to looking straight forward
        if (Math.abs(heading) > Constants.AutoBalancing.max_yaw_offset) {
            // negative so that we move towards the target, not away
            double turning_speed = turnController.calculate(Math.abs(heading), 0);
            turning_speed = turningLimiter.calculate(turning_speed)
                    * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            // remove the negative if it turns the wrong direction around
            turning_speed = Math.abs(turning_speed) * Math.signum(heading);

            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;

            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    0, 0, turning_speed, swerve.getRotation2d());
            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);

            // 6. Output each module states to wheels
            swerve.setModuleStates(moduleStates);
        } else {
            // X is vertical!
            // where we are
            double current_position = swerve.getPose().getX();
            // where we want to be
            // how fast to move
            double speed = driveController.calculate(current_position, start_point + setpoint);
            // and we move
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    speed, 0, 0, swerve.getRotation2d());
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);

            swerve.setModuleStates(moduleStates);
        }

    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();

    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}