package frc.robot.utils;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.Constants.*;

/**
 * @author Advik Garg
 * @author Akhil Singamneni
 * @author Noah Simon
 * The class to manage April Tag Detection and information updates
 */

public class AprilTagGetters {

    private static AprilTagFieldLayout aprilTagFieldLayout;
    static {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            System.out.println(e);
            aprilTagFieldLayout = null;
        }
    }

    // load map field
    private static PhotonCamera camera = new PhotonCamera("OV5647"); // network table that photonvision sends info over

    // CHANGE THE BELOW COMMAND BASED ON WHERE THEY DECIDE
    private static Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    // Translation is for how much forward, left, and up from center the camera is

    private static PhotonPoseEstimator robotPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
    camera, robotToCam);

    public static Optional<EstimatedRobotPose> updatePoseWithAprilTag(Pose2d previousRobotPose) {
        camera.setPipelineIndex(VisionConstants.apriltag_pipeline_index);
        robotPoseEstimator.setReferencePose(previousRobotPose);
        return robotPoseEstimator.update();
 
    }
    

    public static double[] getAprilTagX(PhotonCamera camera) {
        camera.setPipelineIndex(VisionConstants.apriltag_pipeline_index);
        double target_x = 0.0;
        double target_fiducial_id = 0;
        double[] x_and_id = { target_fiducial_id, target_x };
        PhotonPipelineResult result = camera.getLatestResult(); // run this code in perodic (every 20 ms)
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            ; // if else statement for 22code above
            target_x = target.getYaw();
            target_fiducial_id = target.getFiducialId();
            x_and_id[0] = target_fiducial_id;

            x_and_id[1] = target_x;
        }
        return x_and_id;
    }
    // double target_x = target.getYaw(); // Get the yaw position of target
    // double target_y = target.getPitch(); //Get pitch of target
    // double area = target.getArea(); //Get area of target
    // double skew = target.getSkew(); //Get skew of target
    // Transform3d pose = target.getCameraToTarget(); //Tells you info to go to a
    // target
    // int targetID = target.getFiducialId(); //Get id of target
    // double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose,
    // targetPose); //get distance to target
    // Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, targetPose); //get
    // yaw difference between target and robot pose
}
