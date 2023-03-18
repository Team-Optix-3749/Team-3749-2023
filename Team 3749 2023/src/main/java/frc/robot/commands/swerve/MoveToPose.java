package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.SmartData;

public class MoveToPose extends CommandBase {
    private final Swerve swerve;
    private final Pose2d targetPose;

    private double driveErrorAbs;
    private double turnErrorAbs;

    private final ProfiledPIDController driveController = new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
    private final ProfiledPIDController turnController = new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

    private SmartData<Double> turnKP = new SmartData<Double>("Turn KP", 2.6);
    private SmartData<Double> drivekP = new SmartData<Double>("Drive KP", 2.5);

    private SmartData<Double> turnTolerance = new SmartData<Double>("Turn tolerance", 0.1);
    private SmartData<Double> driveTolerance = new SmartData<Double>("Drive tolerance", 0.0);

    public MoveToPose(Swerve swerve, Pose2d targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        addRequirements(swerve);
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        var currentPose = swerve.getPose();
        driveController.reset(
                currentPose.getTranslation().getDistance(targetPose.getTranslation()));
        turnController.reset(currentPose.getRotation().getRadians());
        turnController.setTolerance(turnTolerance.get());
        driveController.setTolerance(driveTolerance.get());
    }

    @Override
    public void execute() {

        driveController.setP(drivekP.get());
        turnController.setP(turnKP.get());

        Pose2d currentPose = swerve.getPose();
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        driveErrorAbs = currentDistance;
        double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
        if (driveController.atGoal())
            driveVelocityScalar = 0.0;

        double turnVelocity = turnController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        turnErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (turnController.atGoal())
            turnVelocity = 0.0;

        Translation2d driveVelocity = new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();
        
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-driveVelocity.getX(), -
        driveVelocity.getY(), turnVelocity, currentPose.getRotation());


        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
            .toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(moduleStates);
            
        SmartDashboard.putNumber("Drive X Velo", driveVelocity.getX());
        SmartDashboard.putNumber("Drive Y Velo", driveVelocity.getY());
        SmartDashboard.putNumber("Turn velo", turnVelocity);
        SmartDashboard.putNumber("Drive error", driveErrorAbs);
        SmartDashboard.putNumber("Turn error", turnErrorAbs);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }

    /** Checks if the robot is stopped at the final pose. */
    public boolean atGoal() {
        return driveController.atGoal() && turnController.atGoal();
    }

    /** Checks if the robot pose is within the allowed drive and theta tolerances. */
    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
        return Math.abs(driveErrorAbs) < driveTolerance
            && Math.abs(turnErrorAbs) < thetaTolerance.getRadians();
    }

    /**
     * Creates a pure translating transform
     *
     * @param x The x componenet of the translation
     * @param y The y componenet of the translation
     * @return The resulting transform
     */
    public static Transform2d translationToTransform(double x, double y) {
        return new Transform2d(new Translation2d(x, y), new Rotation2d());
    }
}
