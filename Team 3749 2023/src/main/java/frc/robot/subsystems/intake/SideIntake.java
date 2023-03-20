package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;

/**
 * Class for controlling the SideIntake lift & intake motors
 * 
 * @author Rohin Sood
 */
public class SideIntake extends SubsystemBase {
    
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.SideIntake.side_intake_id, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    private final CANSparkMax liftMotor = new CANSparkMax(Constants.SideIntake.lift_motor_id, MotorType.kBrushless);
    private final RelativeEncoder liftEncoder = liftMotor.getEncoder();

    private final ArmFeedforward liftFF = new ArmFeedforward(0, Constants.SideIntake.liftKG, 0.0, 0.0);
    private final PIDController liftPID =  new PIDController(Constants.SideIntake.liftKP, 0.0, 0.0);

    private double liftSetpoint = 0.0;

    private ShuffleData<Double> intakeTemp = new ShuffleData<Double>("Side Intake", "Intake Temp (C)", 0.0);
    private ShuffleData<Double> intakeCurrent = new ShuffleData<Double>("Side Intake", "Intake Current", 0.0);
    private ShuffleData<Double> liftPositionRad = new ShuffleData<Double>("Side Intake", "Lift Position (Rad)", 0.0);

    public SideIntake() {
        intakeMotor.restoreFactoryDefaults();
        liftMotor.restoreFactoryDefaults();

        intakeMotor.setIdleMode(IdleMode.kBrake);
        liftMotor.setIdleMode(IdleMode.kCoast);

        intakeEncoder.setPositionConversionFactor(1.0 / 3.0);
        intakeEncoder.setVelocityConversionFactor(1.0 / (60.0 * 5.0));

        liftEncoder.setPositionConversionFactor((2 * Math.PI) / 100.0);
        liftEncoder.setVelocityConversionFactor((2 * Math.PI) / (60.0 * 100.0));

        liftPID.setTolerance(0.1);
    }

    /**
     * gets the temperature of the intake motor
     * 
     * @return
     */
    public double getIntakeTemperature() {
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
        return liftEncoder.getPosition();
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
     * set voltage of life motor
     * @param voltage
     */
    public void setLiftVoltage(double voltage) {
        liftMotor.setVoltage(voltage);
    }

    /**
     * set position (angle in rad) of the lift motor using FF
     * 
     * @param setpoint
     */
    public void setLiftFF(double setpoint) {
        double ff_output = liftFF.calculate(setpoint + Math.PI, 0.0);

        setLiftVoltage(ff_output);
    }

    /**
     * set position (angle in rad) of the lift motor using FF
     * 
     * @param setpoint
     */
    public void setLiftPIDFF(double setpoint) {
        double ff_output = liftFF.calculate(setpoint + Math.PI, 0.0);
        double pid_output = liftPID.calculate(liftEncoder.getPosition(), setpoint);

        setLiftVoltage(pid_output + ff_output);
    }

    /**
     * toggle lift setpoint (Out & In)
     */
    public void toggleLiftSetpoint() {
        liftSetpoint = liftSetpoint == Constants.SideIntake.liftOutSetpoint ? 0.0 : Constants.SideIntake.liftOutSetpoint;
    }

    /**
     * get if at the setpoint
     * 
     * @return if the lift PID loop is at its setpoint
     */
    public boolean liftAtSetpoint() {
        return liftPID.atSetpoint();
    }

    /**
     * set % speed of the motor
     * 
     * @param speed -1.0 to 1.0
     */
    public void setIntake(double speed) {
        // intakeMotor.set(speed);
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
        intakeTemp.set(getIntakeTemperature());
        intakeCurrent.set(intakeMotor.getOutputCurrent());
        liftPositionRad.set(getLiftPosition());

        setLiftPIDFF(liftSetpoint);
    }
}