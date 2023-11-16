package frc.robot.commands.vision;


import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants;
import frc.robot.utils.SmartData;
import frc.robot.utils.Constants.VisionConstants.Pipelines;

/**
 * Using the target Translation2d, drive to the predetermined setpoint using
 * PIDControllers
 * 
 * @author Rohin Sood
 */
public class AlignPiece extends CommandBase {
    private final Swerve swerve;
    private final Limelight limelight;
    private final double setpoint = 0;
    private double error;
    private PhotonTrackedTarget lastTarget;
    private boolean hasTarget = false;
    private SmartData<Double> yKP = new SmartData<Double>("Y KP", .05);

    private final PIDController yController = new PIDController(yKP.get(), 0, 0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            2.7, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

    public AlignPiece() {
        this.swerve = Robot.swerve;
        this.limelight = Robot.limelight;
        this.setName("Piece Align");
        addRequirements(swerve, limelight);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(Pipelines.CUBE.index);

        getError();

        yController.setSetpoint(setpoint);
        yController.setTolerance(0.1);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void execute() {
        yController.setP(yKP.get());

        getError();

        var robotPose2d = swerve.getPose();
        // var robotPose2d = new Pose2d(
        //     swerve.getPose().getTranslation(), new Rotation2d(swerve.getPose().getRotation().getRadians() + Math.PI)
        // );

        if (!hasTarget) {
            return;
        }
        double thetaGoal = DriverStation.getAlliance() == Alliance.Blue ? 0 : Math.PI ;

        double thetaSpeed = thetaController.calculate(
                robotPose2d.getRotation().getRadians(), thetaGoal);
        // thetaSpeed =  DriverStation.getAlliance() == Alliance.Blue ? thetaSpeed : -thetaSpeed;
        
        if (thetaController.atGoal())
            thetaSpeed = 0.0;
        SmartDashboard.putNumber("Theta Speed", thetaSpeed);

        double ySpeed = yController.calculate(error);



        SmartDashboard.putNumber("Y Speed", ySpeed);
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0,
                ySpeed, thetaSpeed, swerve.getRotation2d());
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(moduleStates);

        System.out.println(yController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("hello");
        limelight.setPipeline(Pipelines.APRILTAG.index);
    }

    @Override
    public boolean isFinished() {
        return yController.atSetpoint();
        // return false;
    }

    public void getError() {
        var res = limelight.getLatestResult();
        if (res.hasTargets()) {

            PhotonTrackedTarget target = res.getTargets().stream()
                    .filter(t -> t != lastTarget)
                    .findFirst().get();

            lastTarget = target;

            error = target.getYaw();

            hasTarget = true;

            System.out.println("target found");
        } else {

            hasTarget = false;
            System.out.println("Target not found");

        }
    }

}
