package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private SwerveModuleState theoreticalState = new SwerveModuleState();
    private final PIDController turningPidController;
    private final PIDController drivingPidController;
    private final SimpleMotorFeedforward drivingFeedFordward;

    public SwerveModuleSim() {
        System.out.println("[Init] Creating ModuleIOSim");
        drivingPidController = new PIDController(ModuleConstants.kPDriving, 0, 0);
        drivingFeedFordward = new SimpleMotorFeedforward(ModuleConstants.kSDriving, ModuleConstants.kVDriving);
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    
    private SwerveModuleState getState(ModuleData data) {
        return new SwerveModuleState(data.driveVelocityMPerSec, new Rotation2d(data.turnAbsolutePositionRad));
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
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        // keep our absolute position within 0-2 pi
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }
        // distance traveled + Rad/Time * Time * diameter
        data.drivePositionM = data.drivePositionM
                + (driveSim.getAngularVelocityRadPerSec() * 0.02 * ModuleConstants.kWheelDiameterMeters);
        data.driveVelocityMPerSec = driveSim.getAngularVelocityRadPerSec() * ModuleConstants.kWheelDiameterMeters;
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        data.driveTempCelcius = 0;

        data.turnAbsolutePositionRad = turnAbsolutePositionRad;
        data.turnPositionRad = turnRelativePositionRad;
        data.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
        data.turnTempCelcius = 0;

        data.theoreticalState = theoreticalState;
        data.position = getPosition(data);
    }

    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    @Override
    public void setDesiredState(SwerveModuleState state, ModuleData data) {
        theoreticalState = state;
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState(data).angle);

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