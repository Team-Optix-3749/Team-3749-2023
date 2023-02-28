package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.Constants;

/**
 * Aligns the robot heading using a PIDController
 * 
 * @author Noah Simon
 */
public class AlignHeading extends CommandBase {
    private final Swerve swerve;

    private double heading;
    private boolean has_aligned;

    private final PIDController turnController = new PIDController(0.005, 0.001, 0);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    
    public AlignHeading(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.stopModules();
        has_aligned = false;
    }

    @Override
    public void execute() {
        if (!Constants.withinMargin(Constants.AutoBalancing.max_yaw_offset, heading, 0)) {
            has_aligned = true;
            return;
        }

        // negative so that we move towards the target, not away
        double turning_speed = turnController.calculate(Math.abs(heading), 0);
        turning_speed = turningLimiter.calculate(turning_speed)
                * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        // signs the speed so we move in the correct direction
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
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override 
    public boolean isFinished() {
        return has_aligned;
    }
}
