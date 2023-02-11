
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
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
    
    private final CANSparkMax leftMotor = new CANSparkMax(Constants.Claw.left_side, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(Constants.Claw.right_side, MotorType.kBrushless);

    private final SparkMaxPIDController leftPIDController = leftMotor.getPIDController();
    
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    
    public Claw() {
        rightMotor.restoreFactoryDefaults();
        leftMotor.restoreFactoryDefaults();

        rightMotor.setInverted(true);

        rightMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setIdleMode(IdleMode.kCoast);

        leftPIDController.setP(Constants.Claw.kP.get());
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
     * 
     * @param speed -1.0 to 1.0
     */
    public void set(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    /**
     * sets the desired velocity of the claw using PID
     * 
     * @param velo_setpoint (rotations/sec)
     */
    public void setVeloPID(double velo_setpoint) {
        leftPIDController.setReference(velo_setpoint, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Avg encoder Velo", avgEncoderPos());
    }

}
