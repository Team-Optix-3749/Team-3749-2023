package frc.robot.commands.swerve;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;

import edu.wpi.first.math.controller.PIDController;
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

    private final Swerve swerve;

    private final PIDController controller = new PIDController(0.004, 0.001, 0.00075);

    private double angle;
    private double heading;
    private boolean has_aligned;
    private boolean past_center;
    private double max_angle = 0;
    private double start_time_balanced = 0;
    private double goalHeading = 0;

    // Initializes the BaseCommand
    public 
    AutoBalancingPID(Swerve swerve, double goalHeading) {
        this.swerve = swerve;
        this.goalHeading = goalHeading;
        addRequirements(swerve);

    }

    // Run on command init
    @Override
    public void initialize() {
        swerve.stopModules();
        // update start position when we are getting started
        angle = swerve.getVerticalTilt();
        has_aligned = false;
        past_center = false;
        max_angle = 0;
        start_time_balanced = 0;

    }

    // Run every 20 ms
    @Override
    public void execute() {

        // if (!Constants.withinMargin(Constants.AutoBalancing.max_yaw_offset, Math.abs(heading), goalHeading)) {
        //     swerve.turnToRotation(goalHeading);
        // }

        heading = swerve.getRotation2d().getDegrees();
        angle = swerve.getVerticalTilt();


        // the robot must've moved slightly past the center now, so we will start using
        // PID to reach the middle
        if (!Constants.withinMargin(Constants.AutoBalancing.max_pitch_offset, angle, 0)) {
            System.out.println("BALANCING");
            has_aligned = true;
            start_time_balanced = 0;
            double speed = -controller.calculate(angle, 0)
                    * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    speed, 0, 0, swerve.getRotation2d());
            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);
            // 6. Output each module states to wheels
            swerve.setModuleStates(moduleStates);
        }
        // we level? Stop with wheels facing different directions to prevent sliding
        else {
            SwerveModuleState[] states = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                states[i] = new SwerveModuleState(0, new Rotation2d(45 + 90 * i));
            }
            swerve.setModuleStates(states);

            start_time_balanced = Timer.getFPGATimestamp();
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
        if (start_time_balanced != 0) {
            if (Timer.getFPGATimestamp() - start_time_balanced > 2) {
                return true;
            }
        }
        return false;
    }

}