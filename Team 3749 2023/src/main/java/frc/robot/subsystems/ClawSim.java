package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ClawSim extends SubsystemBase {
    /*
     * distance per pulse = (angle per revolution) / (pulses per revolution)
     * = (2 * PI rads) / (4096 pulses)
     * 
     * DPP = dist per pulse
     */

	private static final double encoder_DPP = (2.0 * Math.PI) / 4096;

    // Creates a PIDController with gains kP, kI, and kD
    PIDController claw_PID = new PIDController(Constants.Claw.claw_kP, Constants.Claw.claw_kI, Constants.Claw.claw_kD);

    private CANSparkMax neo = new CANSparkMax(Constants.Base.neo_id, MotorType.kBrushless);

    private MotorControllerGroup base = new MotorControllerGroup(neo);

    // Initializes the clawSim subsystem
    public ClawSim() {
        neo.setInverted(true);
        //Constants.Base.speed.set(new Double(16.90));
    }

    /***
     * Sets the speed. Value is between -1.0 and 1.0
     * 
     * @param percent_speed
     */
    public void set(double percent_speed) {
        base.set(percent_speed);
    }

    /***
     * Gets the speed. Value is between -1.0 and 1.0
     * 
     * @return speed of the first motor in the motor controller group (neo)
     */
    public double get() {
        return base.get();
    }

    // Runs every 20 ms
    @Override
    public void periodic() {

    }

}