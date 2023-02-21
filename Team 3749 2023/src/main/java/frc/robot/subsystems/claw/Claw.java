package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

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

    private final SparkMaxPIDController clawMotorPIDController = clawMotor.getPIDController();

    public Claw() {
        clawMotor.restoreFactoryDefaults();

        clawMotor.setIdleMode(IdleMode.kBrake);
        clawMotor.setSmartCurrentLimit(60);
        clawMotorPIDController.setP(Constants.Claw.kP.get());
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

    /**
     * stops the motor
     */
    public void stop() {
        clawMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Claw Temp (C)", getTemperature());
        SmartDashboard.putNumber("Claw Position", getPosition());
        SmartDashboard.putNumber("Claw Current", clawMotor.getOutputCurrent());

        // clawMotor.setSmartCurrentLimit(Constants.Claw.currentLimit.get().intValue(),
        // 5700);
        set(0.1);
    }
}
