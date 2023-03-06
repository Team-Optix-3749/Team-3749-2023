package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/***
 * @author Rohin Sood
 */
public class SideIntake extends SubsystemBase {
    
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.SideIntake.side_intake_id, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    private final CANSparkMax liftMotor = new CANSparkMax(Constants.SideIntake.lift_motor_id, MotorType.kBrushless);
    private final RelativeEncoder liftEncoder = liftMotor.getEncoder();

    private final ArmFeedforward liftFF = new ArmFeedforward(0, Constants.SideIntake.liftKG.get(), 0.0, 0.0);

    public SideIntake() {
        intakeMotor.restoreFactoryDefaults();
        liftMotor.restoreFactoryDefaults();

        intakeMotor.setIdleMode(IdleMode.kBrake);
        liftMotor.setIdleMode(IdleMode.kBrake);

        intakeEncoder.setPositionConversionFactor(1.0 / 3.0);
        intakeEncoder.setVelocityConversionFactor(1.0 / (60.0 * 5.0));

        liftEncoder.setPositionConversionFactor(1.0 / 100.0);
        liftEncoder.setVelocityConversionFactor((Math.PI * 2) / (60.0 * 100.0));
    }

    /**
     * gets the temperature of the intake motor
     * 
     * @return
     */
    public double getClawTemperature() {
        return intakeMotor.getMotorTemperature();
    }

    /**
     * gets the temperature of the intake motor
     * 
     * @return
     */
    public double getIntakePosition() {
        return intakeEncoder.getPosition();
    }

    /**
     * gets the temperature of the lift motor
     * 
     * @return
     */
    public double getLiftPosition() {
        return intakeEncoder.getPosition();
    }

    /**
     * gets the velocity of the intake motor
     * 
     * @return
     */
    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }

    /**
     * set voltage of motor
     * 
     * @param voltage
     */
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    /**
     * set position of the lift motor
     * 
     * @param voltage
     */
    public void setLiftPosition(double position) {
        liftEncoder.setPosition(position);
    }

    /**
     * set position of the lift motor using FF
     * 
     * @param voltage
     */
    public void setLiftFF(double setpoint) {
        double ff_output = liftFF.calculate(setpoint, 0.0);

        SmartDashboard.putNumber("Lift FF Ouptut", ff_output);

        setLiftPosition(ff_output);
    }

    /**
     * set % speed of the motor
     * 
     * @param speed -1.0 to 1.0
     */
    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * stops the motor
     */
    public void stop() {
        intakeMotor.stopMotor();
        liftMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Side Intake Temp (C)", getClawTemperature());
        SmartDashboard.putNumber("Side Intake Position", intakeEncoder.getPosition());
        SmartDashboard.putNumber("Side Intake Velocity", intakeEncoder.getVelocity());
        SmartDashboard.putNumber("Side Intake Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Side Intake Voltage", intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
        SmartDashboard.putString("Side Intake Command",
                this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
    }
}