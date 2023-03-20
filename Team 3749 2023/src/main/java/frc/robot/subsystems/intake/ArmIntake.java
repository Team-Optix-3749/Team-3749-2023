package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;

/***
 * @author Anusha Khobare
 * @author Aashray Reddy
 * @author Ryan R McWeeny
 * @author Hanlun Li
 * @author Harkirat Hattar
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 * 
 *         ArmIntake.java creates objects, dependencies, and motor controller groups
 *         to allow us to set the speed of each motor for intake and outtake
 */
public class ArmIntake extends SubsystemBase {

    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.ArmIntake.arm_intake_id, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    private final PIDController intakePID = new PIDController(0.675, 0, 0);
    private final SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(0, 0.675);

    private final ShuffleData<Double> intakeVoltage = new ShuffleData<Double>("Arm Intake", "Intake Voltage", 0.0);
    private final ShuffleData<Double> intakeCurrent = new ShuffleData<Double>("Arm Intake", "Intake Current", 0.0);
    private final ShuffleData<Double> intakeTemp = new ShuffleData<Double>("Arm Intake", "Intake Temperature", 0.0);

    public ArmIntake() {
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setIdleMode(IdleMode.kBrake);

        intakeMotor.setInverted(true);

        // 1 wheel rotation / 5 motor rotations
        intakeEncoder.setPositionConversionFactor(1.0 / 5.0);

        // 1 minute / 60 seconds * 1 wheel rotation / 5 motor rotations
        intakeEncoder.setVelocityConversionFactor(1.0 / (60.0 * 5.0));
    }

    /**
     * gets the temperature of the claw motor
     * 
     * @return
     */
    public double getTemperature() {
        return intakeMotor.getMotorTemperature();
    }

    /**
     * gets the temperature of the claw motor
     * 
     * @return
     */
    public double getPosition() {
        return intakeEncoder.getPosition();
    }

    /**
     * set voltage of motor
     * 
     * @param voltage
     */
    public void setVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    /**
     * set % speed of the motor
     * 
     * @param speed -1.0 to 1.0
     */
    public void set(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * set claw motor using feed forward control loop
     * 
     * @param velocity
     */
    public void setFeedForward(double velocity) {
        intakeMotor.setVoltage(
                intakeFF.calculate(velocity) + intakePID.calculate(intakeEncoder.getVelocity(), velocity));
    }

    /**
     * stops the motor
     */
    public void stop() {
        intakeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        intakeTemp.set(getTemperature());
        intakeCurrent.set(intakeMotor.getOutputCurrent());
        intakeVoltage.set(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    }
}