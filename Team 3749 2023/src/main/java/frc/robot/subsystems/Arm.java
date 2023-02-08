package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.BruteInverseKinematics;

/**
 * @author Bailey Say
 * @author Raymond Sheng
 * @author Don Tran
 * 
 * Code for the arm of the robot. This does not include the claw at the end
 * 
 */

public class Arm extends SubsystemBase {
    private CANSparkMax leftShoulderMotor = new CANSparkMax(Constants.Arm.left_shoulder_id, MotorType.kBrushless);
    private CANSparkMax rightShoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);

    private CANSparkMax leftElbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private CANSparkMax rightElbowMotor = new CANSparkMax(Constants.Arm.right_elbow_id, MotorType.kBrushless);

    // Relative Encoder Initialization (relative for now)
    // For simplicity, left encoder is designated as shoulder/elbow encoder for now
    private final RelativeEncoder shoulderEncoder = leftShoulderMotor.getEncoder();
    // private final RelativeEncoder rightShoulderEncoder = rightShoulderMotor.getEncoder();

    private final RelativeEncoder elbowEncoder = leftElbowMotor.getEncoder();
    // private final RelativeEncoder rightElbowEncoder = rightElbowMotor.getEncoder();

    // PIDs (to change on the fly in smartdashboard, might need to put this in the arm command)
    // TODO: confirm max velocity/acceleration constraints
    private ProfiledPIDController elbowController =  new ProfiledPIDController(Constants.Arm.elbowKP.get(), Constants.Arm.elbowKI.get(), Constants.Arm.elbowKD.get(),
        new TrapezoidProfile.Constraints(2, 5));
    private ProfiledPIDController shoulderController = new ProfiledPIDController(Constants.Arm.shoulderKP.get(), Constants.Arm.shoulderKI.get(), Constants.Arm.shoulderKD.get(),
        new TrapezoidProfile.Constraints(2, 5));

    public Arm() {
        // invert right motors
        rightElbowMotor.setInverted(true);
        rightShoulderMotor.setInverted(true);

        // right motors are follower motors for left motors
        rightShoulderMotor.follow(leftShoulderMotor);
        rightElbowMotor.follow(leftElbowMotor);

        // conversion factor is ((gear ratio)/(encoder resolution) * 360) degrees
        leftShoulderEncoder.setPositionConversionFactor(250/2048*360);
        leftElbowEncoder.setPositionConversionFactor(250/2048*360);
    }

    // Sets speed of a motors
    public void setSpeedElbow(double speed) {
        leftElbowMotor.set(speed);
    }

    public void setSpeedShoulder(double speed) {
        leftShoulderMotor.set(speed);
    }

    // Sets voltage of motors
    public void setElbowVoltage(double voltage) {
        leftElbowMotor.setVoltage(voltage);
    }

    public void setShoulderVoltage(double voltage) {
        leftShoulderMotor.setVoltage(voltage);
    }

    public void setDegreesShoulder(double x, double y) {
        // NEED PID TO CONTROL ACCURATELY
        // would work if the conversion factor was correctly set
        // need to change where it gets the calculated angle (x and y vals)
        // leftShoulderEncoder.setPosition(shoulderController.calculate(leftShoulderEncoder.getPosition(), BruteInverseKinematics.calculate(x, y)[1]));
    }

    public void setDegreesElbow(double x, double y) {
        // NEED PID TO CONTROL ACCURATELY
        // would work if the conversion factor was correctly set
        // need to change where it gets the calculated angle (x and y vals)
        // leftElbowEncoder.setPosition(elbowController.calculate(leftElbowEncoder.getPosition(), BruteInverseKinematics.calculate(x, y)[1]));
    }

    public ProfiledPIDController getElbowController() {
        return elbowController;
    }

    public ProfiledPIDController getShoulderController() {
        return shoulderController;
    }

    public RelativeEncoder getElbowEncoder() {
        return elbowEncoder;
    }

    public RelativeEncoder getShoulderEncoder() {
        return shoulderEncoder;
    }
}