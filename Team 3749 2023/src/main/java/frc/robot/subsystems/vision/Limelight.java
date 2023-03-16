package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.VisionConstants;
import frc.robot.utils.Constants.VisionConstants.Nodes;

/**
 * Encapsulated PhotonCamera object used in posed estimation and alignment
 * 
 * @author Rohin Sood
 */
public class Limelight extends SubsystemBase{

    private final  PhotonCamera camera = new PhotonCamera("limelight");
    private  PhotonPipelineResult result = getLatestResult();
    private AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, Constants.VisionConstants.robot_to_cam);

    public Limelight() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    public  PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public  boolean hasTarget(PhotonPipelineResult result) {
        return result.hasTargets();
    }

    public  List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
        return result.getTargets();
    }

    public  PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        return result.getBestTarget();
    }

    public  Rotation2d getYaw(PhotonTrackedTarget target) {
        return new Rotation2d(Math.toRadians(target.getYaw()));
    }

    public  double getPitch(PhotonTrackedTarget target) {
        return target.getPitch();
    }

    public  double getArea(PhotonTrackedTarget target) {
        return target.getArea();
    }

    public  double getSkew(PhotonTrackedTarget target) {
        return target.getSkew();
    }

    public  List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
        return target.getDetectedCorners();
    }

    public  int getTargetId(PhotonTrackedTarget target) {
        return target.getFiducialId();
    }

    public  double getPoseAbmiguity(PhotonTrackedTarget target) {
        return target.getPoseAmbiguity();
    }

    public  double getDistance(PhotonTrackedTarget target, VisionConstants.Nodes node) {
        return PhotonUtils.calculateDistanceToTargetMeters(
                Constants.VisionConstants.camera_height,
                node.height,
                Constants.VisionConstants.camera_pitch,
                Units.degreesToRadians(getPitch(target)));
    }

    public  Translation2d getTranslation2d(PhotonTrackedTarget target, VisionConstants.Nodes node) {
        return PhotonUtils.estimateCameraToTargetTranslation(
                getDistance(target, node), getYaw(target));
    }

    public  int getPipeline() {
        return camera.getPipelineIndex();
    }

    public  void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    public  void setLED(VisionLEDMode ledMode) {
        camera.setLED(ledMode);
    }

    public void updatePoseAprilTags(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
        var result = getLatestResult();
        var filter = result.getTargets().stream()
                .filter(t -> t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                .findFirst();
        if (filter.isPresent()) {
            var target = filter.get();
            var imageCaptureTime = result.getTimestampSeconds();
            var camToTargetTrans = target.getBestCameraToTarget();
            var camPose = Constants.VisionConstants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
            SmartDashboard.putNumber("CAM POSE X", camPose.getX());
            SmartDashboard.putNumber("CAM POSE Y", camPose.getY());
            SmartDashboard.putNumber("CAM POSE Z", camPose.getZ());
            swerveDrivePoseEstimator.addVisionMeasurement(
                getEstimatedGlobalPose(swerveDrivePoseEstimator.getEstimatedPosition()).get().estimatedPose.toPose2d(), imageCaptureTime);
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public  void logging() {

        result = getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            SmartDashboard.putNumber("target pitch: ", getPitch(target));
            SmartDashboard.putNumber("target yaw (degrees): ", getYaw(target).getDegrees());
            SmartDashboard.putNumber("target distance: ",
                    getDistance(target, Constants.VisionConstants.Nodes.MID_CUBE));
            SmartDashboard.putNumber("Target translation 2d X: ",
                    getTranslation2d(target, Constants.VisionConstants.Nodes.MID_CUBE).getX());
            SmartDashboard.putNumber("Target translation 2d Y: ",
                    getTranslation2d(target, Constants.VisionConstants.Nodes.MID_CUBE).getY());
        }
    }

    @Override
    public void periodic() {

    }

}