package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.SmartData;

/***
 * @author Anusha Khobare
 * @author Aashray Reddy
 * @author Ryan R McWeeny
 * @author Hanlun Li
 * @author Harkirat Hattar
 * 
 *         Claw.java creates objects, dependencies, and motor controller groups
 *         to allow us to set the speed of each motor for intake and outtake
 */

public class Claw extends SubsystemBase {

    private final CANSparkMax clawMotor = new CANSparkMax(Constants.Claw.claw_id, MotorType.kBrushless);
    private final RelativeEncoder clawEncoder = clawMotor.getEncoder();
    private SmartData<Double> kv = new SmartData<Double>("claw kv", 0.0775);
    private SmartData<Double> kp = new SmartData<Double>("claw kp", 0.3);

    private final PIDController clawPID = new PIDController(kp.get(), 0, 0);
    private final SimpleMotorFeedforward clawFeedForward = new SimpleMotorFeedforward(0, kv.get());

    public Claw() {
        clawMotor.restoreFactoryDefaults();

        clawMotor.setIdleMode(IdleMode.kBrake);
        clawMotor.setSmartCurrentLimit(50);
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
     * set % speed of the motor
     * 
     * @param speed -1.0 to 1.0
     */
    public void set(double speed) {
        clawMotor.set(speed);
    }

    public void setFeedForward(double velocity) {
        SmartDashboard.putNumber("claw pid gain",clawPID.calculate(clawEncoder.getVelocity(),velocity));
        SmartDashboard.putNumber("claw ff gain",clawFeedForward.calculate(velocity));

        SmartDashboard.putNumber("claw gain", clawFeedForward.calculate(velocity) + clawPID.calculate(clawEncoder.getVelocity(),velocity));
        clawMotor.set(clawFeedForward.calculate(velocity) + clawPID.calculate(velocity));

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
        // clawMotor.setSmartCurrentLimit(Constants.Claw.currentLimit.get().intValue(),
        // 5700);
        // set(0.1);
    }
}
