//Claw.java creates objects, dependencies, and motor controller groups
//to allow us to set the speed of each motor for intake and outtake

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Claw extends SubsystemBase {
    // Creates a PIDController with gains kP, kI, and kD
    PIDController claw_PID = new PIDController(Constants.Claw.claw_kP, Constants.Claw.claw_kI, Constants.Claw.claw_kD);

    // right side of the claw (the motor)
    private CANSparkMax right_motor = new CANSparkMax(Constants.Claw.right_side, MotorType.kBrushless);

    // left side of the claw (the motor)
    private CANSparkMax left_motor = new CANSparkMax(Constants.Claw.left_side, MotorType.kBrushless);

    // motor controller group for both sides
    private MotorControllerGroup clawMotors = new MotorControllerGroup(left_motor, right_motor);

    private final RelativeEncoder claw_encoder = right_motor.getEncoder();

    // Initializes the base subsystem
    public Claw() {
        right_motor.setInverted(true); // invert the motor to not break it

        right_motor.setIdleMode(IdleMode.kBrake); // set neo to be braked when not active
        left_motor.setIdleMode(IdleMode.kBrake); // set neo to be braked when not active

        // not sure what this is: Constants.Base.speed.set(new Double(16.90));
    }

    /**
     * set speed for motor
     * 
     * @param speed
     */
    public void setSpeed(double speed) {
        clawMotors.set(speed);
    }

    // Runs every 20 ms
    @Override
    public void periodic() {
        // Calculates the output of the PID algorithm based on the sensor reading
        // and sends it to a motor
        right_motor.set(claw_PID.calculate(claw_encoder.getPosition(), Constants.Claw.setpoint));
        left_motor.set(claw_PID.calculate(claw_encoder.getPosition(), Constants.Claw.setpoint));
    }

}
