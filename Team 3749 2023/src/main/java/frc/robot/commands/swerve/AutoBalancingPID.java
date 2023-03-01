package frc.robot.commands.swerve;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Noah Simon
 * 
 *         Allows the robot to automatically "engage" on the charging station
 *         Strategy: Move forward until change in angle. Then use PID from the
 *         angle measurement to balance
 */
public class AutoBalancingPID extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    Swerve swerveSubsystem;

    private final PIDController controller = new PIDController(0.003, 0, 0.00075);
    private final PIDController turnController = new PIDController(0.005, 0.001, 0);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    private double angle;
    private double heading;
    private boolean has_aligned;
    private boolean past_center;
    private double max_angle = 0;
    private double start_time_balanced = 0;

    // Initializes the BaseCommand
    public AutoBalancingPID(Swerve swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    // Run on command init
    @Override
    public void initialize() {
        swerveSubsystem.stopModules();
        // update start position when we are getting started
        angle = swerveSubsystem.getVerticalTilt();
        has_aligned = false;
        past_center = false;
        max_angle = 0;
        start_time_balanced = 0;

    }

    // Run every 20 ms
    @Override
    public void execute() {

        heading = swerveSubsystem.getHeading();
        angle = swerveSubsystem.getVerticalTilt();
        if (Math.abs(angle) > Math.abs(max_angle)) {
            max_angle = angle;
        }
        if (Math.abs(max_angle) - 7 > Math.abs(angle)) {
            past_center = true;
        }

        // How inaccurate we are willing to be in reference to looking straight forward
        // Should change this so it adjusts on the go and doesn't need to stop
        if (!Constants.withinMargin(Constants.AutoBalancing.max_yaw_offset, heading, 0) && !has_aligned) {
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
                    0, 0, turning_speed, swerveSubsystem.getRotation2d());
            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);
            // 6. Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        }

        // move forward if the angle hasn't started to move and it hasn't moved in the
        // past
        else if (!Constants.withinMargin(Constants.AutoBalancing.max_pitch_offset, angle, 0) && !past_center) {
            has_aligned = true;
            start_time_balanced = 0;
            // Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    Constants.AutoBalancing.base_speed_mps, 0, 0, swerveSubsystem.getRotation2d());
            // Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);
            // Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        }
        // the robot must've moved slightly past the center now, so we will start using
        // PID to reach the middle
        else if (!Constants.withinMargin(Constants.AutoBalancing.max_pitch_offset, angle, 0)) {
            has_aligned = true;
            start_time_balanced = 0;

            double speed = -controller.calculate(angle, 0)
                    * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    speed, 0, 0, swerveSubsystem.getRotation2d());
            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);
            // 6. Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        }
        // we level? Stop with wheels facing different directions to prevent sliding
        else {
            SwerveModuleState[] states = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                states[i] = new SwerveModuleState(0, new Rotation2d(45 + 90 * i));
            }
            swerveSubsystem.setModuleStates(states);

            start_time_balanced = Timer.getFPGATimestamp();
        }

    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        if (start_time_balanced != 0) {
            if (Timer.getFPGATimestamp() - start_time_balanced > 2) {
                return true;
            }
        }
        return false;
    }

}