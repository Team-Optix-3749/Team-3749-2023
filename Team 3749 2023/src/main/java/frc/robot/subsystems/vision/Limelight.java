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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.VisionConstants;

/**
 * Encapsulated PhotonCamera object used in posed estimation and alignment
 * 
 * @author Rohin Sood
 */
public class Limelight extends SubsystemBase {

    private final PhotonCamera camera = new PhotonCamera("limelight");
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;

    private final NetworkTable photonTable = NetworkTableInstance.getDefault().getTable("photonvision");
    private final NetworkTableEntry ledMode = photonTable.getEntry("ledMode");

    public Limelight() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    camera, Constants.VisionConstants.robot_to_cam);
        } catch (Exception e) {
            System.out.println(e);
        }

        setLED(VisionLEDMode.kOff);
    }

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

    public double getDistance(PhotonTrackedTarget target, VisionConstants.Nodes node) {
        return PhotonUtils.calculateDistanceToTargetMeters(
                Constants.VisionConstants.camera_height,
                node.height,
                Constants.VisionConstants.camera_pitch,
                Units.degreesToRadians(getPitch(target)));
    }

    public Translation2d getTranslation2d(PhotonTrackedTarget target, VisionConstants.Nodes node) {
        return PhotonUtils.estimateCameraToTargetTranslation(
                getDistance(target, node), getYaw(target));
    }
    
    public AprilTagFieldLayout getAprilTagFieldLayout(){
        return aprilTagFieldLayout;
    }

    public int getPipeline() {
        return camera.getPipelineIndex();
    }

    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    public void setLED(VisionLEDMode ledMode) {

        switch (ledMode) {
            case kOn:
                this.ledMode.setInteger(1);
                break;
            case kOff:
                this.ledMode.setInteger(0);
                break;
            case kBlink:
                this.ledMode.setInteger(2);
                break;
            default:
                this.ledMode.setInteger(-1);
                break;
        }
        camera.setLED(ledMode);
    }

    public void updatePoseAprilTags(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
        var result = getLatestResult();
        var filter = result.getTargets().stream()
                .filter(t -> t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                .findFirst();
        if (filter.isPresent()) {
            var imageCaptureTime = result.getTimestampSeconds();
            Optional<EstimatedRobotPose> poseEstimate = getEstimatedGlobalPose(
                    swerveDrivePoseEstimator.getEstimatedPosition());
            // if it recieved a pose update
            if (poseEstimate.isPresent()) {
                Pose2d newPose = poseEstimate.get().estimatedPose.toPose2d();
                // if the update is sufficiantly different to the current one, done to not cause pid oscilation
                if (!Constants.withinMargin(0.05, newPose.getTranslation(), swerveDrivePoseEstimator.getEstimatedPosition().getTranslation())){
                    swerveDrivePoseEstimator.addVisionMeasurement(
                        newPose, imageCaptureTime);
                }
                // swerveDrivePoseEstimator.setVisionMeasurementStdDevs(null);
            }
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public void logging() {

        var result = getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            SmartDashboard.putNumber("target pitch: ", getPitch(target));
            SmartDashboard.putNumber("target yaw (degrees): ", getYaw(target).getDegrees());
            SmartDashboard.putNumber("Target translation 2d X: ",
                    getTranslation2d(target, Constants.VisionConstants.Nodes.MID_CONE).getX());
            SmartDashboard.putNumber("Target translation 2d Y: ",
                    getTranslation2d(target, Constants.VisionConstants.Nodes.MID_CONE).getY());
        }
    }

    @Override
    public void periodic() {

        logging();
    }

}