package frc.robot.commands.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.Limelight;

/***
 * @author Noah Simon
 * @author Raadwan _____
 * @author Rohin Sood
 *         Default command to control the SwervedriveSubsystem with joysticks
 */
public class VisionDefaultCommand extends CommandBase {

    SwerveDrivePoseEstimator swervePoseEstimator;
    Limelight limelight;

    public VisionDefaultCommand(Limelight limelight, SwerveDrivePoseEstimator swervePoseEstimator) {
        this.swervePoseEstimator = swervePoseEstimator;
        this.limelight = limelight;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        limelight.setPipeline(0);
        limelight.updatePoseAprilTags(swervePoseEstimator);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}