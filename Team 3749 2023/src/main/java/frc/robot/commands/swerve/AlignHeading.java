package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;

/**
 * Aligns the robot heading using a PIDController
 * 
 * @author Noah Simon
 */
public class AlignHeading extends CommandBase {
    private final Swerve swerve;

    private double heading;

    private final PIDController turnController = new PIDController(0.005, 0.0001, 0);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    public AlignHeading(Swerve swerve) {
        this.swerve = swerve;
        this.turnController.enableContinuousInput(-180,180);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.stopModules();
        turnController.setTolerance(2.2);
        turnController.setSetpoint(0.0);
    }

    @Override
    public void execute() {
        heading = swerve.getHeading();

        System.out.println("margin " + Constants.withinMargin(Constants.AutoBalancing.max_yaw_offset, heading, 0.0));
        System.out.println("turn " + turnController.atSetpoint());
        System.out.println("turn error" + turnController.getPositionError());
        System.out.println("turn setpoint" + turnController.getSetpoint());

        if (atGoal())
            return;

        // negative so that we move towards the target, not away
        double turning_speed = turnController.calculate(Math.abs(heading));

        turning_speed = turningLimiter.calculate(turning_speed)
                * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        // signs the speed so we move in the correct direction
        turning_speed = Math.abs(turning_speed) * Math.signum(heading);

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
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
        System.out.println("tang finished");
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }

    public boolean atGoal() {
        return Constants.withinMargin(Constants.AutoBalancing.max_yaw_offset, heading, 0.0)
                || turnController.atSetpoint();
    }
}
