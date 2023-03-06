package frc.robot.subsystems.ground;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/***
 * @author Rohin Sood
 */
public class GroundClaw extends SubsystemBase {

    private final CANSparkMax groundClawMotor = new CANSparkMax(Constants.GroundClaw.ground_claw_id, MotorType.kBrushless);
    private final RelativeEncoder groundClawEncoder = groundClawMotor.getEncoder();

    private final SimpleMotorFeedforward clawFeedForward = new SimpleMotorFeedforward(0, 0.675);

    public GroundClaw() {
        groundClawMotor.restoreFactoryDefaults();

        groundClawMotor.setIdleMode(IdleMode.kBrake);
        // clawMotor.setSmartCurrentLimit(60);

        // 1 wheel rotation / 5 motor rotations
        groundClawEncoder.setPositionConversionFactor(1.0 / 5.0);

        // 1 minute / 60 seconds * 1 wheel rotation / 5 motor rotations
        groundClawEncoder.setVelocityConversionFactor(1.0 / (60.0 * 5.0));
    }

    /**
     * gets the temperature of the claw motor
     * 
     * @return
     */
    public double getTemperature() {
        return groundClawMotor.getMotorTemperature();
    }

    /**
     * gets the temperature of the claw motor
     * 
     * @return
     */
    public double getPosition() {
        return groundClawEncoder.getPosition();
    }

    /**
     * set voltage of motor
     * 
     * @param voltage
     */
    public void setVoltage(double voltage) {
        groundClawMotor.setVoltage(voltage);
    }

    /**
     * set % speed of the motor
     * 
     * @param speed -1.0 to 1.0
     */
    public void set(double speed) {
        groundClawMotor.set(speed);
    }

    /**
     * stops the motor
     */
    public void stop() {
        groundClawMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ground Claw Temp (C)", getTemperature());
        SmartDashboard.putNumber("Ground Claw Position", groundClawEncoder.getPosition());
        SmartDashboard.putNumber("Ground Claw Velocity", groundClawEncoder.getVelocity());
        SmartDashboard.putNumber("Ground Claw Current", groundClawMotor.getOutputCurrent());
        SmartDashboard.putNumber("Ground Claw Voltage", groundClawMotor.getAppliedOutput() * groundClawMotor.getBusVoltage());
        SmartDashboard.putString("Ground Claw Command",
                this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
    }
}