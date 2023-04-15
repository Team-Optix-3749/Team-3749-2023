package frc.robot.commands.vision;
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
import frc.robot.utils.Constants.VisionConstants.Node;

/**
 * Using the target Translation2d, drive to the predetermined setpoint using
 * PIDControllers
 * 
 * @author Rohin Sood
 */
public class AlignRetro extends CommandBase {
    private final Swerve swerve;
    private final Limelight limelight;
    private final Node node;

    private final Translation2d setpoint;

    private PhotonTrackedTarget lastTarget;

    private SmartData<Double> xKP = new SmartData<Double>("X KP", 1.0);
    private SmartData<Double> yKP = new SmartData<Double>("Y KP", 1.0);

    private final PIDController xController = new PIDController(0.0, 0, 0);
    private final PIDController yController = new PIDController(0.0, 0, 0);
    private final PIDController turnController = new PIDController(2.6, 0, 0);

    public AlignRetro(Swerve swerve, Limelight limelight, Node node) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.node = node;
        this.setName("Vision Align");
        setpoint = node == Node.TOP_CONE ? new Translation2d(1.3, 0.1) : new Translation2d(1.3, 0.1);

        turnController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve, limelight);
    }

    @Override
    public void initialize() {
        limelight.setLED(VisionLEDMode.kOn);
        /**
         * TODO: set the target grouping in PhotonVision to lower in MID_CONE and higher in TOP_CONE
         */
        // limelight.setPipeline(
        //     node == Node.TOP_CONE ? Pipelines.TOP_CONE.index : Pipelines.MID_CONE.index
        // );

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
            if (targetList.size() > 1 || targetList.isEmpty()) {
                System.out.println("targets detected: " + targetList.size());
                return;
            }

            var target = targetList.get(0);

            lastTarget = target;

            var targetTranslation = limelight.getTranslation2d(target, node);
            
            
            double xSpeed = xController.calculate(targetTranslation.getX());
            double ySpeed = yController.calculate(targetTranslation.getY());
            double thetaSpeed = turnController.calculate(swerve.getHeading());

            SmartDashboard.putNumber("Target 2d X", targetTranslation.getX());
            SmartDashboard.putNumber("Target 2d Y", targetTranslation.getY());

            SmartDashboard.putNumber("X Speed", xSpeed);
            SmartDashboard.putNumber("Y Speed", ySpeed);
            SmartDashboard.putNumber("theta Speed", thetaSpeed);

            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed,
                    ySpeed, thetaSpeed, new Rotation2d(Units.degreesToRadians(swerve.getHeading())));
            SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                    .toSwerveModuleStates(chassisSpeeds);

            swerve.setModuleStates(moduleStates);
        } else {
            System.out.println("Reflective tape not found");
        }
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setLED(VisionLEDMode.kOff);
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }

    public boolean atGoal() {
        return xController.atSetpoint() && yController.atSetpoint();
    }

}
