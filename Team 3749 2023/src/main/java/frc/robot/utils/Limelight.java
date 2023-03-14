package frc.robot.utils;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.VisionConstants;

/**
 * Encapsulated PhotonCamera object used in posed estimation and alignment
 * 
 * @author Rohin Sood
 */
public class Limelight {

    private static final PhotonCamera camera = new PhotonCamera("limelight");
    private static PhotonPipelineResult result = getLatestResult();

    public static PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public static boolean hasTarget(PhotonPipelineResult result) {
        return result.hasTargets();
    }

    public static List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
        return result.getTargets();
    }

    public static PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        return result.getBestTarget();
    }

    public static Rotation2d getYaw(PhotonTrackedTarget target) {
        return new Rotation2d(Math.toRadians(target.getYaw()));
    }

    public static double getPitch(PhotonTrackedTarget target) {
        return target.getPitch();
    }

    public static double getArea(PhotonTrackedTarget target) {
        return target.getArea();
    }

    public static double getSkew(PhotonTrackedTarget target) {
        return target.getSkew();
    }

    public static List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
        return target.getDetectedCorners();
    }

    public static int getTargetId(PhotonTrackedTarget target) {
        return target.getFiducialId();
    }

    public static double getPoseAbmiguity(PhotonTrackedTarget target) {
        return target.getPoseAmbiguity();
    }

    public static double getDistance(PhotonTrackedTarget target, VisionConstants.Nodes node) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            Constants.VisionConstants.camera_height,
            node.height,
            Constants.VisionConstants.camera_pitch,
            Units.degreesToRadians(getPitch(target)));
    }

    public static Translation2d getTranslation2d(PhotonTrackedTarget target, VisionConstants.Nodes node) {
        return PhotonUtils.estimateCameraToTargetTranslation(
            getDistance(target, node), getYaw(target)
        ); 
    }

    public static int getPipeline() {
        return camera.getPipelineIndex();
    }

    public static void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    public static void setLED(VisionLEDMode ledMode) {
        camera.setLED(ledMode);
    }

    public static void updatePoseAprilTags(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
        var result = getLatestResult();
        if (result.hasTargets()) {
            var imageCaptureTime = result.getTimestampSeconds();
            var camToTargetTrans = result.getBestTarget().getBestCameraToTarget();
            var camPose = Constants.VisionConstants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
            SmartDashboard.putNumber("CAM POSE X", camPose.getX());
            SmartDashboard.putNumber("CAM POSE Y", camPose.getY());
            SmartDashboard.putNumber("CAM POSE Z", camPose.getZ());
            swerveDrivePoseEstimator.addVisionMeasurement(
                    camPose.toPose2d(), imageCaptureTime);
        }
    }

    public static void logging() {

        result = getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            SmartDashboard.putNumber("target pitch: ", getPitch(target));
            SmartDashboard.putNumber("target yaw (degrees): ", getYaw(target).getDegrees());
            SmartDashboard.putNumber("target distance: ", getDistance(target, Constants.VisionConstants.Nodes.MID_CUBE));
            SmartDashboard.putNumber("Target translation 2d X: ", getTranslation2d(target, Constants.VisionConstants.Nodes.MID_CUBE).getX());
            SmartDashboard.putNumber("Target translation 2d Y: ", getTranslation2d(target, Constants.VisionConstants.Nodes.MID_CUBE).getY());
        }
    }

}