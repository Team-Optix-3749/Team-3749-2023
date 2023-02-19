
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
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
    
    private final CANSparkMax rightMotor = new CANSparkMax(Constants.Claw.right_side, MotorType.kBrushless);
    private final CANSparkMax leftMotor = new CANSparkMax(Constants.Claw.left_side, MotorType.kBrushless);
    
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    
    private final PIDController clawPID = new PIDController(Constants.Claw.claw_kP, Constants.Claw.claw_kI,
            Constants.Claw.claw_kD);
    
    public Claw() {
        rightMotor.setInverted(true);
        rightMotor.follow(leftMotor);

        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * averages the Encoder velocities from both left and right encoders.(gives
     * error if method is not set to return double)
     * 
     * @return
     */
    public double avgEncoderPos() {
        double encoder_avg = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
        return encoder_avg;
    }

    /**
     * set % speed of the motor
     */
    public void set(double speed) {
        leftMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Avg encoder Velo", avgEncoderPos());
    }

}