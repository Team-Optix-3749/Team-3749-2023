package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Vision extends SubsystemBase {

    private final PhotonCamera camera = new PhotonCamera("photonvision");
    private PhotonPipelineResult result = getLatestResult();

    public PhotonPipelineResult getLatestResult() {
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

    public Rotation2d getYaw(PhotonTrackedTarget target) {
        return new Rotation2d(Math.toRadians(target.getYaw()));
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

    public double getDistance(PhotonTrackedTarget target) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            Constants.VisionConstants.camera_height,
            Constants.VisionConstants.camera_yaw,
            Constants.VisionConstants.camera_pitch,
            Units.degreesToRadians(getPitch(target)));
    }

    public Translation2d getTranslation2d(PhotonTrackedTarget target) {
        return PhotonUtils.estimateCameraToTargetTranslation(
            getDistance(target), getYaw(target)
        ); 
    }

    public int getPipeline() {
        return camera.getPipelineIndex();
    }

    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    public void setLED(VisionLEDMode ledMode) {
        camera.setLED(ledMode);
    }

    @Override
    public void periodic() {
        result = getLatestResult();
        if (result.hasTargets()) {
            System.out.println("hello");
            PhotonTrackedTarget target = result.getBestTarget();
            SmartDashboard.putNumber("target pitch: ", getPitch(target));
            SmartDashboard.putNumber("target yaw (degrees): ", getYaw(target).getDegrees());
            SmartDashboard.putNumber("target distance: ", getDistance(target));
        }
    }

}