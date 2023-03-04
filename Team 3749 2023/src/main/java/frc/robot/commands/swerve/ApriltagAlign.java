package frc.robot.commands.swerve;

import frc.robot.utils.Limelight;
import frc.robot.utils.Constants.VisionConstants;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Adapted from 7028's ChaseTagCommand
 * 
 * @author Rohin Sood
 */
public class ApriltagAlign extends CommandBase {

    private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(1, 2);
    private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(1, 2);
    private static final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(1, 2);

    private static final int tag_fid_id = 1;
    private static final Transform3d tagToGoal = new Transform3d(
            new Translation3d(-1.5, 0.0, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI));

    private final Swerve swerve;

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, xConstraints);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, yConstraints);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, 0, thetaConstraints);

    private PhotonTrackedTarget lastTarget;

    public ApriltagAlign(Swerve swerve) {
        this.swerve = swerve;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        thetaController.setTolerance(Units.degreesToRadians(3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        var robotPose = swerve.getPose();
        thetaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        var robotPose2d = swerve.getPose();
        var robotPose = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        var photonRes = Limelight.getLatestResult();
        if (photonRes.hasTargets()) {
            // Find the tag we want to chase
            var targetOpt = photonRes.getTargets().stream()
                    .filter(t -> t.getFiducialId() == tag_fid_id)
                    .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                    .findFirst();
            if (targetOpt.isPresent()) {
                var target = targetOpt.get();
                // This is new target data, so recalculate the goal
                lastTarget = target;

                // Transform the robot's pose to find the camera's pose
                var cameraPose = robotPose.transformBy(VisionConstants.robot_to_cam);

                // Trasnform the camera's pose to the target's pose
                var camToTarget = target.getBestCameraToTarget();
                var targetPose = cameraPose.transformBy(camToTarget);

                // Transform the tag's pose to set our goal
                var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

                // Drive
                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                thetaController.setGoal(goalPose.getRotation().getRadians());
            }
        }

        if (lastTarget == null) {
            // No target has been visible
            swerve.stop();
        } else {
            // Drive to the target
            var xSpeed = xController.calculate(robotPose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }

            var ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }

            var thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians());
            if (thetaController.atGoal()) {
                thetaSpeed = 0;
            }

            SmartDashboard.putNumber("X Speed", xSpeed);
            SmartDashboard.putNumber("Y Speed", ySpeed);
            SmartDashboard.putNumber("TEHTA Speed", thetaSpeed);

            swerve.drive(xSpeed, ySpeed, thetaSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}