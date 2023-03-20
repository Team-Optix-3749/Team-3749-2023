package frc.robot.commands.swerve;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants;
import frc.robot.utils.SmartData;
import frc.robot.utils.Constants.VisionConstants.Nodes;

/**
 * Aligns the robot with the tracked target and drives to its required distance
 * for placing
 * 
 * @author Rohin Sood
 */
public class RetroAlign extends CommandBase {
    private final Swerve swerve;
    private final Limelight limelight;

    private final Translation2d setpoint = new Translation2d(1.3, 0.1);

    private PhotonTrackedTarget lastTarget;

    private SmartData<Double> xKP = new SmartData<Double>("X KP", 1.0);
    private SmartData<Double> yKP = new SmartData<Double>("Y KP", 1.0);

    private final PIDController xController = new PIDController(0.0, 0, 0);
    private final PIDController yController = new PIDController(0.0, 0, 0);

    public RetroAlign(Swerve swerve, Limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(swerve);
        this.setName("Vision Align");
    }

    @Override
    public void initialize() {
        limelight.setLED(VisionLEDMode.kOn);

        xController.setSetpoint(setpoint.getX());
        xController.setTolerance(0.1);
        
        yController.setSetpoint(setpoint.getY());
        yController.setTolerance(0.1);
    }

    @Override
    public void execute() {
        xController.setP(xKP.get());
        yController.setP(yKP.get());

        var res = limelight.getLatestResult();
        if (res.hasTargets()) {

            Stream<PhotonTrackedTarget> targetStream = res.getTargets().stream()
                    .filter(t -> t != lastTarget);

            var targetList = targetStream.collect(Collectors.toList());

            // if more than one target is found, its obviously not detecting pieces of
            // reflective tape that should be on the field and the rest of the command
            // should not be run
            if (targetList.size() > 1) {
                System.out.println("Detecting too many targets");
                return;
            }

            if (!targetList.isEmpty()) {

                var target = targetList.get(0);

                lastTarget = target;

                var targetTranslation = limelight.getTranslation2d(target, Nodes.TOP_CONE);

                SmartDashboard.putNumber("Target X", targetTranslation.getX());
                SmartDashboard.putNumber("Target Y", targetTranslation.getY());

                // getX() is vertical, getY() is horizontal
                double xSpeed = xController.calculate(targetTranslation.getX());
                double ySpeed = yController.calculate(targetTranslation.getY());

                SmartDashboard.putNumber("X Speed", xSpeed);
                SmartDashboard.putNumber("Y Speed", ySpeed);

                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed,
                        ySpeed, 0.0, new Rotation2d(Units.degreesToRadians(swerve.getHeading())));
                SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                        .toSwerveModuleStates(chassisSpeeds);

                swerve.setModuleStates(moduleStates);
            }
        } else {
            System.out.println("Reflective tape not found");
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("FINSIDHIFHSIDGFAJKSDGFKASDHF");
        limelight.setLED(VisionLEDMode.kOff);
    }

    @Override
    public boolean isFinished() {
        System.out.println("FINSIDHIFHSIDGFAJKSDGFKASDHF");
        return atGoal();
    }

    public boolean atGoal() {
        return xController.atSetpoint() && yController.atSetpoint();
    }

}
