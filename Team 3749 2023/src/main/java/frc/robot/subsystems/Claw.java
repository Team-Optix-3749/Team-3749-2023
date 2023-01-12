package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/***
 * @author Rohin Sood
 * 
 *         Serves as a template to format subsystems
 */
public class Claw extends SubsystemBase {

    private CANSparkMax neo = new CANSparkMax(Constants.Base.neo_id, MotorType.kBrushless);
    private WPI_TalonFX falcon = new WPI_TalonFX(Constants.Base.falcon_id);

    private MotorControllerGroup base = new MotorControllerGroup(neo, falcon);

    CANSparkMax rightSide = new CANSparkMax(Constants.right_side, MotorType.kBrushless);
    // right side of the claw (the motor)

    CANSparkMax leftSide = new CANSparkMax(Constants.left_side, MotorType.kBrushless);
    // left side of the claw (the motor)

    // Initializes the base subsystem
    public Claw() {
        neo.setInverted(true); // invert the motor to not break it
        neo.setIdleMode(IdleMode.kBrake); // set neo to be braked when not active

        // set falcon to be braked when not active
        // equivalent to neo.setIdleMode(IdleMode.kBrake)
        falcon.setNeutralMode(NeutralMode.Brake);

        // not sure what this is: Constants.Base.speed.set(new Double(16.90));
    }

    // two motor controller groups allow us to alter the speeds between each motor
    // (on the claw)
    // this is important because both motors have to spin opposite directions
    private MotorControllerGroup left = new MotorControllerGroup(leftSide);
    private MotorControllerGroup right = new MotorControllerGroup(rightSide);

    // now set the speed of each motor (they will be the same but inverted)
    public void setSpeed(double right_speed, double left_speed) {
        right.set(right_speed);
        left.set(left_speed);
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
