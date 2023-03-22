package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Map;
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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.VisionConstants;
import frc.robot.utils.Constants.VisionConstants.Node;

/**
 * Encapsulated PhotonCamera object used in posed estimation and alignment
 * 
 * @author Rohin Sood
 */
public class Limelight extends SubsystemBase {

    private final PhotonCamera camera = new PhotonCamera("limelight");
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    private final NetworkTable photonTable = NetworkTableInstance.getDefault().getTable("photonvision");
    private final NetworkTableEntry ledMode = photonTable.getEntry("ledMode");
    private final NetworkTableEntry ledModeState = photonTable.getEntry("ledModeState");
    private final NetworkTableEntry ledModeRequest = photonTable.getEntry("ledModeRequest");

    private final HttpCamera raw;
    private final HttpCamera processed;

    private final ShuffleData<Double> targetPitch = new ShuffleData<Double>("Limelight", "Target Pitch", 0.0);
    private final ShuffleData<Double> targetYaw = new ShuffleData<Double>("Limelight", "Target Yaw", 0.0);
    private final ShuffleData<Double> targetTranslationX = new ShuffleData<Double>("Limelight",
            "Target Translation2d X (MID)", 0.0);
    private final ShuffleData<Double> targetTranslationY = new ShuffleData<Double>("Limelight",
            "Target Translation2d Y (MID)", 0.0);
    private final ShuffleData<Integer> pipeline = new ShuffleData<Integer>("Limelight",
            "Pipeline", -1000);

    public Limelight() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    camera, Constants.VisionConstants.robot_to_cam);
        } catch (Exception e) {
            System.out.println(e);
        }

        raw = new HttpCamera("Raw", "http://photonvision.local:1181/stream.mjpg?1679365741443");
        processed = new HttpCamera("Processed", "http://photonvision.local:1182/stream.mjpg?1679365741443stream");
        
        CameraServer.addCamera(raw);
        CameraServer.addCamera(processed);

        // add HttpCameras
        limelightTab.add(raw).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0)
                .withSize(5, 4).withProperties(Map.of("Show controls", false));
        limelightTab.add(processed).withWidget(BuiltInWidgets.kCameraStream).withPosition(6, 0)
                .withSize(5, 4).withProperties(Map.of("Show controls", false));

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

    public double getDistance(PhotonTrackedTarget target, Node node) {
        return PhotonUtils.calculateDistanceToTargetMeters(
                Constants.VisionConstants.camera_height,
                node.height_meters,
                Constants.VisionConstants.camera_pitch,
                Units.degreesToRadians(getPitch(target)));
    }

    public Translation2d getTranslation2d(PhotonTrackedTarget target, Node node) {
        return PhotonUtils.estimateCameraToTargetTranslation(
                getDistance(target, node), getYaw(target));
    }

    public AprilTagFieldLayout getAprilTagFieldLayout() {
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
                ledModeState.setInteger(1);
                ledModeRequest.setInteger(1);
                break;
            case kOff:
                this.ledMode.setInteger(0);
                ledModeState.setInteger(0);
                ledModeRequest.setInteger(0);
                break;
            case kBlink:
                this.ledMode.setInteger(2);
                ledModeState.setInteger(2);
                ledModeRequest.setInteger(2);
                break;
            default:
                this.ledMode.setInteger(-1);
                ledModeState.setInteger(-1);
                ledModeRequest.setInteger(-1);
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
                // if the update is sufficiantly different to the current one, done to not cause
                // pid oscilation
                if (!Constants.withinMargin(0.05, newPose.getTranslation(),
                        swerveDrivePoseEstimator.getEstimatedPosition().getTranslation())) {
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
            var target = getBestTarget(getLatestResult());

            targetPitch.set(getPitch(target));
            targetYaw.set(getYaw(target).getDegrees());

            targetTranslationX.set(getTranslation2d(target, Node.MID_CONE).getX());
            targetTranslationY.set(getTranslation2d(target, Node.MID_CONE).getY());
        }

        pipeline.set(getPipeline());

    }

    @Override
    public void periodic() {
        logging();
    }

}