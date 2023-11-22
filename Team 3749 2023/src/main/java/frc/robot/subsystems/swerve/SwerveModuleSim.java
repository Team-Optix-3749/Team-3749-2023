package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.Sim;

/* 
Very closely inspired by 6328's Swerve Sim code,
 https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/drive/ModuleIOSim.java
*/
public class SwerveModuleSim implements SwerveModuleIO {
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    private double turnPositionRad = 0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private SwerveModuleState theoreticalState = new SwerveModuleState();
    private final PIDController turningPidController;
    private final PIDController drivingPidController;
    private final SimpleMotorFeedforward drivingFeedFordward;

    public SwerveModuleSim() {
        System.out.println("[Init] Creating ModuleIOSim");
        drivingPidController = new PIDController(ModuleConstants.kPDrivingReal, 0, 0);
        drivingFeedFordward = new SimpleMotorFeedforward(ModuleConstants.kSDrivingReal, ModuleConstants.kVDrivingReal);
        turningPidController = new PIDController(ModuleConstants.kPTurningReal, 0, 0);
        turningPidController.enableContinuousInput(0, 2 * Math.PI);
    }

    private SwerveModuleState getState() {
        return new SwerveModuleState(
                driveSim.getAngularVelocityRadPerSec() * Constants.ModuleConstants.kWheelDiameterMeters / 2,
                new Rotation2d(turnPositionRad));
    }

    private SwerveModulePosition getPosition(ModuleData data) {
        return new SwerveModulePosition(data.drivePositionM, new Rotation2d(data.turnAbsolutePositionRad));
    }

    @Override
    public void updateData(ModuleData data) {
        // update sim values
        driveSim.update(Sim.loopPeriodSec);
        turnSim.update(Sim.loopPeriodSec);

        // how far have we turned in the previous loop?
        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Sim.loopPeriodSec;
        // update our angle variables
        turnPositionRad += angleDiffRad;
        // keep our absolute position within 0-2 pi
        while (turnPositionRad < 0) {
            turnPositionRad += 2.0 * Math.PI;
        }
        while (turnPositionRad > 2.0 * Math.PI) {
            turnPositionRad -= 2.0 * Math.PI;
        }
        // distance traveled + Rad/Time * Time * diameter
        data.drivePositionM = data.drivePositionM
                + (driveSim.getAngularVelocityRadPerSec() * 0.02 * ModuleConstants.kWheelDiameterMeters) / 2;
        data.driveVelocityMPerSec = driveSim.getAngularVelocityRadPerSec() * ModuleConstants.kWheelDiameterMeters / 2;
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        data.driveTempCelcius = 0;

        data.turnAbsolutePositionRad = turnPositionRad;
        data.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
        data.turnTempCelcius = 0;

        data.theoreticalState = theoreticalState;
        data.position = getPosition(data);
    }

    private void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    private void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        turnSim.setInputVoltage(turnAppliedVolts);
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