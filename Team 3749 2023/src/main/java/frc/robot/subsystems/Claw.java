//Claw.java creates objects, dependencies, and motor controller groups
//to allow us to set the speed of each motor for intake and outtake

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Claw extends SubsystemBase {


    // right side of the claw (the motor)
    private CANSparkMax rightSide = new CANSparkMax(Constants.right_side, MotorType.kBrushless);

    // left side of the claw (the motor)
    private CANSparkMax leftSide = new CANSparkMax(Constants.left_side, MotorType.kBrushless);

    // Initializes the base subsystem
    public Claw() {
        rightSide.setInverted(true); // invert the motor to not break it

        rightSide.setIdleMode(IdleMode.kBrake); // set neo to be braked when not active
        leftSide.setIdleMode(IdleMode.kBrake); // set neo to be braked when not active

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
        left.set(percent_speed);
        right.set(percent_speed);
    }

    /***
     * Gets the speed. Value is between -1.0 and 1.0
     * 
     * @return speed of the first motor in the motor controller group (neo)
     */
    public double get() {
        return left.get();
        //return right.get();
    }

    // Runs every 20 ms
    @Override
    public void periodic() {

    }

}
