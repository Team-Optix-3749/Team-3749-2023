package frc.robot.commands.vision;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Constants;
import frc.robot.utils.SmartData;
import frc.robot.utils.Constants.VisionConstants.Piece;
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
    private final Piece piece;

    private final double setpoint = 0;

    private double error;

    private PhotonTrackedTarget lastTarget;

    private boolean hasTarget = false;

    private SmartData<Double> yKP = new SmartData<Double>("Y KP", .025);

    private final PIDController yController = new PIDController(yKP.get(), 0, 0);

    public AlignPiece(Swerve swerve, Limelight limelight, Piece piece) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.piece = piece;
        this.setName("Piece Align");
        addRequirements(swerve, limelight);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(
            piece == Piece.CONE ? Pipelines.CONE.index : Pipelines.CUBE.index
        );

        getError();

        yController.setSetpoint(setpoint);
        yController.setTolerance(0.04);
    }

    @Override
    public void execute() {
        yController.setP(yKP.get());

        if (!hasTarget){
            getError();
        }

        System.out.println("ALIGN EXECUTE");

        double ySpeed = yController.calculate(error);

        SmartDashboard.putNumber("Y Speed", ySpeed);
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0,
                -ySpeed, 0, new Rotation2d(Units.degreesToRadians(swerve.getHeading())));
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(moduleStates);

        System.out.println(yController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("hello");
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
        } else {
            System.out.println("Target not found");

        }
    }

}
