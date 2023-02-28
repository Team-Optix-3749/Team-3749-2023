package frc.robot.commands.swerve;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.VisionConstants;

/**
 * Aligns the robot with the tracked target and drives to its required distance for placing
 * 
 * @author Rohin Sood
 */
public class VisionAlign extends CommandBase {
    private final Swerve swerve;
    private final Vision vision;
    private final VisionConstants.Nodes node;
    private final PIDController xController = new PIDController(VisionConstants.visionXKP.get(), 0, 0);
    private final PIDController yController = new PIDController(VisionConstants.visionYKP.get(), 0, 0);
    private boolean aligned;

    public VisionAlign(Vision vision, Swerve swerve, VisionConstants.Nodes node) {
        this.vision = vision;
        this.swerve = swerve;
        this.node = node;
    }

    @Override
    public void initialize() {
        if (node == VisionConstants.Nodes.MID_CONE || node == VisionConstants.Nodes.TOP_CONE)
            vision.setPipeline(VisionConstants.Pipelines.REFLECTIVE_TAPE.index);
        else
            vision.setPipeline(VisionConstants.Pipelines.APRILTAG.index);

        vision.setLED(VisionLEDMode.kOn);
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target;
        if (vision.hasTarget(vision.getLatestResult()))
            target = vision.getBestTarget(vision.getLatestResult());
        else
            return;

        Translation2d relativeTargetPose = vision.getTranslation2d(target);

        aligned = (new Translation2d(VisionConstants.camera_offset, node.dist) == swerve.getPose().getTranslation());

        double xSpeed = xController.calculate(relativeTargetPose.getX(), VisionConstants.camera_offset);
        double ySpeed = yController.calculate(relativeTargetPose.getY(), node.dist);

        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Y Speed", ySpeed);

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, 0, swerve.getRotation2d());
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);

        // swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        vision.setLED(VisionLEDMode.kOff);
    }

    @Override
    public boolean isFinished() {
        return aligned;
    }
}
