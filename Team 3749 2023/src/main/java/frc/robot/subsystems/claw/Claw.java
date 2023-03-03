package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/***
 * @author Anusha Khobare
 * @author Aashray Reddy
 * @author Ryan R McWeeny
 * @author Hanlun Li
 * @author Harkirat Hattar
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 *         /**
 *         Double jointed arm subsystem built with 2 CANSparkMaxes at each joint
 *         and REV
 *         Through Bore Encoders
 * 
 *         Claw.java creates objects, dependencies, and motor controller groups
 *         to allow us to set the speed of each motor for intake and outtake
 */
public class Claw extends SubsystemBase {

    private final CANSparkMax clawMotor = new CANSparkMax(Constants.Claw.claw_id, MotorType.kBrushless);
    private final RelativeEncoder clawEncoder = clawMotor.getEncoder();

    private final PIDController clawPID = new PIDController(0.675, 0, 0);
    private final SimpleMotorFeedforward clawFeedForward = new SimpleMotorFeedforward(0, 0.675);

    public Claw() {
        clawMotor.restoreFactoryDefaults();

        clawMotor.setIdleMode(IdleMode.kBrake);
        // clawMotor.setSmartCurrentLimit(60);

        // 1 wheel rotation / 5 motor rotations
        clawEncoder.setPositionConversionFactor(1.0 / 5.0);

        // 1 minute / 60 seconds * 1 wheel rotation / 5 motor rotations
        clawEncoder.setVelocityConversionFactor(1.0 / (60.0 * 5.0));
    }

    /**
     * gets the temperature of the claw motor
     * 
     * @return
     */
    public double getTemperature() {
        return clawMotor.getMotorTemperature();
    }

    /**
     * gets the temperature of the claw motor
     * 
     * @return
     */
    public double getPosition() {
        return clawEncoder.getPosition();
    }

    /**
     * set voltage of motor
     * 
     * @param voltage
     */
    public void setVoltage(double voltage) {
        clawMotor.setVoltage(voltage);
    }

    /**
     * set % speed of the motor
     * 
     * @param speed -1.0 to 1.0
     */
    public void set(double speed) {
        clawMotor.set(speed);
    }

    /**
     * set claw motor using feed forward control loop
     * 
     * @param velocity
     */
    public void setFeedForward(double velocity) {
        SmartDashboard.putNumber("claw pid gain", clawPID.calculate(clawEncoder.getVelocity(), velocity));
        SmartDashboard.putNumber("claw ff gain", clawFeedForward.calculate(velocity));

        SmartDashboard.putNumber("claw gain",
                clawFeedForward.calculate(velocity) + clawPID.calculate(clawEncoder.getVelocity(), velocity));
        clawMotor.setVoltage(
                clawFeedForward.calculate(velocity) + clawPID.calculate(clawEncoder.getVelocity(), velocity));
    }

    /**
     * stops the motor
     */
    public void stop() {
        clawMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Claw Temp (C)", getTemperature());
        SmartDashboard.putNumber("Claw Position", clawEncoder.getPosition());
        SmartDashboard.putNumber("Claw Velocity", clawEncoder.getVelocity());
        SmartDashboard.putNumber("Claw Current", clawMotor.getOutputCurrent());
        SmartDashboard.putNumber("Claw Voltage", clawMotor.getAppliedOutput() * clawMotor.getBusVoltage());
        SmartDashboard.putString("Claw Command",
                this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
    }
}
