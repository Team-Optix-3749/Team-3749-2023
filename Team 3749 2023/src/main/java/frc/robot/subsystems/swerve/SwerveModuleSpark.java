package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.Sim;

/* 
Very closely inspired by 6328's Swerve Sim code,
 https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/drive/ModuleIOSim.java
*/
public class SwerveModuleSpark implements SwerveModuleIO {

    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor = new CANSparkMax(0, MotorType.kBrushless);
    RelativeEncoder driveEncoder = driveMotor.getEncoder();

    private final CANCoder absoluteEncoder = new CANCoder(0);


    private double turnPositionRad = 0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private SwerveModuleState theoreticalState = new SwerveModuleState();
    private final PIDController turningPidController;
    private final PIDController drivingPidController;
    private final SimpleMotorFeedforward drivingFeedFordward;

    public SwerveModuleSpark(int index) {
        driveMotor = new CANSparkMax(DriveConstants.driveMotorPorts[index], MotorType.kBrushless);

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMotorGearRatio);

        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);


        System.out.println("[Init] Creating ModuleIOSim");
        drivingPidController = new PIDController(ModuleConstants.kPDrivingReal, 0, 0);
        drivingFeedFordward = new SimpleMotorFeedforward(ModuleConstants.kSDrivingReal, ModuleConstants.kVDrivingReal);
        turningPidController = new PIDController(ModuleConstants.kPTurningReal, 0, 0);
        turningPidController.enableContinuousInput(0, 2 * Math.PI);
    }

    private SwerveModuleState getState() {
        return new SwerveModuleState(
               driveEncoder.getVelocity(),
                new Rotation2d(turnPositionRad));
    }

    private SwerveModulePosition getPosition(ModuleData data) {
        return new SwerveModulePosition(data.drivePositionM, new Rotation2d(data.turnAbsolutePositionRad));
    }

    @Override
    public void updateData(ModuleData data) {

        // how far have we turned in the previous loop?
        // distance traveled + Rad/Time * Time * diameter
        data.drivePositionM = driveEncoder.getPosition()/6.75;
        data.driveVelocityMPerSec = driveEncoder.getVelocity()/6.75;
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveMotor.getOutputCurrent());
        data.driveTempCelcius = driveMotor.getMotorTemperature();


        turnPositionRad = absoluteEncoder.getAbsolutePosition() / 180 * Math.PI;
        data.turnAbsolutePositionRad = turnPositionRad;
        data.turnVelocityRadPerSec = absoluteEncoder.getVelocity() / 180 * Math.PI;
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnMotor.getOutputCurrent());
        data.turnTempCelcius = turnMotor.getMotorTemperature();

        data.theoreticalState = theoreticalState;
        data.position = getPosition(data);
    }

    private void setDriveVoltage(double volts) {
        // driveAppliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        driveAppliedVolts = volts;
        SmartDashboard.putNumber("Drive volts", volts);
        // driveSim.setInputVoltage(driveAppliedVolts);
        driveMotor.setVoltage(volts);
    }

    private void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        // turnSim.setInputVoltage(turnAppliedVolts);
        turnMotor.setVoltage(volts);
    }

    @Override
    public void setDesiredState(SwerveModuleState state, ModuleData data) {
        state = SwerveModuleState.optimize(state, getState().angle);

        theoreticalState = state;
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        double drive_volts = drivingFeedFordward.calculate(state.speedMetersPerSecond)
                + drivingPidController.calculate(data.driveVelocityMPerSec, state.speedMetersPerSecond);

        double turning_volts = turningPidController.calculate(data.turnAbsolutePositionRad, state.angle.getRadians());
        // Make a drive PID Controller
        setDriveVoltage(drive_volts);
        setTurnVoltage(turning_volts);

    }

    @Override
    public void stop() {
        setDriveVoltage(0);
        setTurnVoltage(0);
    }
}