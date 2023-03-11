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
import frc.robot.utils.Constants;
import frc.robot.utils.Limelight;
import frc.robot.utils.Constants.VisionConstants;

/**
 * Aligns the robot with the tracked target and drives to its required distance for placing
 * 
 * @author Rohin Sood
 */
public class RetroAlign extends CommandBase {
    private final Swerve swerve;
    private final VisionConstants.Nodes node;
    private final PIDController xController = new PIDController(VisionConstants.visionXKP.get(), 0, 0);
    private final PIDController yController = new PIDController(VisionConstants.visionYKP.get(), 0, 0);
    private boolean aligned;
    private double offset;

    public RetroAlign(Swerve swerve, VisionConstants.Nodes node) {
        this.swerve = swerve;
        this.node = node;
        this.setName("Vision Align");
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("vision align", "init");
        if (node == VisionConstants.Nodes.MID_CONE || node == VisionConstants.Nodes.TOP_CONE) {
            Limelight.setPipeline(VisionConstants.Pipelines.REFLECTIVE_TAPE.index);
            offset = VisionConstants.retro_cam_offset;
        } else {
            Limelight.setPipeline(VisionConstants.Pipelines.APRILTAG.index);
            offset = VisionConstants.apriltag_cam_offset;
        }

        Limelight.setLED(VisionLEDMode.kOn);

        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("vision align", "exectue");
        SmartDashboard.putBoolean("at setpoint", xController.atSetpoint());

        PhotonTrackedTarget target;
        if (Limelight.hasTarget(Limelight.getLatestResult())) {
            target = Limelight.getBestTarget(Limelight.getLatestResult());
        } else {
            return;
        }

        Translation2d cameraToTargetTranslation = Limelight.getTranslation2d(target, node);

        aligned = (VisionConstants.retro_cam_offset == swerve.getPose().getTranslation().getX());

        double xSpeed = xController.calculate(cameraToTargetTranslation.getX(), offset);
        double ySpeed = yController.calculate(cameraToTargetTranslation.getY(), node.height);

        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Y Speed", ySpeed);

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, xSpeed, 0, swerve.getRotation2d());
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(moduleStates);

        xController.setP(VisionConstants.visionXKP.get());
        yController.setP(VisionConstants.visionYKP.get());
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("vision align", "end");

        Limelight.setLED(VisionLEDMode.kOff);
    }

    @Override
    public boolean isFinished() {
        return aligned;
    }
}
