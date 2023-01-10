package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.DrivetrainOld;


/***
 * The Command for running Swerve Drive during the Teleop portion of the match
 *  @author Noah Simon
 *  @author Harkirat
 *  @see https://www.youtube.com/watch?v=0Xi9yb1IMyA
 */
public class SwerveTeleopOld extends CommandBase {

    private final DrivetrainOld swerveSubsystem;
    private final DoubleSupplier xSpdFunction, ySpdFunction, turningSpdFunction;
    private final BooleanSupplier fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    /***
     * 
     * @param swerveSubsystem Intakes the instance of the swerve subsystem
     * @param xSpdFunction The lambda to recieve x speed
     * @param ySpdFunction The lambda to recieve x speed
     * @param turningSpdFunction The lambda to recieve turning speed
     * @param fieldOrientedFunction The lambda to recieve whether or not to orient on the filed 
     */
    // public SwerveTeleop(Drivetrain swerveSubsystem,
    //         Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
    //         Supplier<Boolean> fieldOrientedFunction) {
    public SwerveTeleopOld(DrivetrainOld swerveSubsystem,
        DoubleSupplier xSpdFunction, DoubleSupplier ySpdFunction, DoubleSupplier turningSpdFunction,
        BooleanSupplier fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(Constants.DrivetrainOld.tele_drive_max_acceleration_units_per_second);
        this.yLimiter = new SlewRateLimiter(Constants.DrivetrainOld.tele_drive_max_acceleration_units_per_second);
        this.turningLimiter = new SlewRateLimiter(Constants.DrivetrainOld.tele_drive_max_angular_acceleration_units_per_second);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    /***
     * Calculates speeds, creates swerve module states, and sets the subsystem to those states
     */
    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.getAsDouble();
        double ySpeed = ySpdFunction.getAsDouble();
        double turningSpeed = turningSpdFunction.getAsDouble();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.DrivetrainOld.deadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.DrivetrainOld.deadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.DrivetrainOld.deadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother by limiting acceleration
        xSpeed = xLimiter.calculate(xSpeed) * Constants.DrivetrainOld.tele_drive_max_speed_meters_per_second;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DrivetrainOld.tele_drive_max_speed_meters_per_second;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.DrivetrainOld.tele_drive_max_angular_acceleration_units_per_second;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.getAsBoolean()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        }
        
        else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.DrivetrainOld.driveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}