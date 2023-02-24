package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable table = instance.getTable("photonvision");

    private final PhotonCamera camera = new PhotonCamera("photonvision");
    private PhotonPipelineResult result = getLatestPipeline();

    public PhotonPipelineResult getLatestPipeline() {
        return camera.getLatestResult();
    }

    public boolean hasTarget(PhotonPipelineResult result) {
        return result.hasTargets();
    }

    public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
        return result.getTargets();
    }

    public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        return result.getBestTarget();
    }

    public double getYaw(PhotonTrackedTarget target) {
        return target.getYaw();
    }

    public double getPitch(PhotonTrackedTarget target) {
        return target.getPitch();
    }

    public double getArea(PhotonTrackedTarget target) {
        return target.getArea();
    }

    public double getSkew(PhotonTrackedTarget target) {
        return target.getSkew();
    }

    public List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
        return target.getDetectedCorners();
    }

    public int getTargetId(PhotonTrackedTarget target) {
        return target.getFiducialId();
    }

    public double getPoseAbmiguity(PhotonTrackedTarget target) {
        return target.getPoseAmbiguity();
    }

    @Override
    public void periodic() {
        result = getLatestPipeline();
        if (result.hasTargets()) {
            SmartDashboard.putNumber("target pitch: ", result.getBestTarget().getPitch());
            SmartDashboard.putNumber("target yaw: ", result.getBestTarget().getYaw());
        }

    }

}