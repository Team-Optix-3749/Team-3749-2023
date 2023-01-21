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
    
    //motor controller group for both sides
    private MotorControllerGroup clawMotors = new MotorControllerGroup(leftSide, rightSide);

    // Initializes the base subsystem
    public Claw() {
        rightSide.setInverted(true); // invert the motor to not break it

        rightSide.setIdleMode(IdleMode.kBrake); // set neo to be braked when not active
        leftSide.setIdleMode(IdleMode.kBrake); // set neo to be braked when not active

        // not sure what this is: Constants.Base.speed.set(new Double(16.90));
    }

    /**
     * set the speed for the motors
     * @param right_speed
     * @param left_speed
     * 
     */
    public void setSpeed(double speed) {
        clawMotors.set(speed);
    }

}
