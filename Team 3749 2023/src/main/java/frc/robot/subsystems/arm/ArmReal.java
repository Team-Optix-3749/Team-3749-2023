package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;

public class ArmReal extends Arm {
  
  private final CANSparkMax leftShoulderMotor = new CANSparkMax(Constants.Arm.left_shoulder_id, MotorType.kBrushless);
	private final CANSparkMax rightShoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);
	private final DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(0);
	private PIDController shoulderPIDController = new PIDController(0.008, 0, 0);

	private final CANSparkMax leftElbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
	private final CANSparkMax rightElbowMotor = new CANSparkMax(Constants.Arm.right_elbow_id, MotorType.kBrushless);
	private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(1);
	private PIDController elbowPIDController = new PIDController(0.008, 0, 0);

	private ProfiledPIDController elbowController = new ProfiledPIDController(Constants.Arm.elbowKP.get(),
			Constants.Arm.elbowKI.get(), Constants.Arm.elbowKD.get(),
			new TrapezoidProfile.Constraints(2, 5));
	private ProfiledPIDController shoulderController = new ProfiledPIDController(Constants.Arm.shoulderKP.get(),
			Constants.Arm.shoulderKI.get(), Constants.Arm.shoulderKD.get(),
			new TrapezoidProfile.Constraints(2, 5));

  public ArmReal() {
    leftShoulderMotor.restoreFactoryDefaults();
    rightShoulderMotor.restoreFactoryDefaults();

    leftElbowMotor.restoreFactoryDefaults();
    rightElbowMotor.restoreFactoryDefaults();
  }

  @Override
  public void setElbowVoltage(double voltage) {
		leftElbowMotor.setVoltage(voltage);
	}

  @Override
	public void setShoulderVoltage(double voltage) {
		leftShoulderMotor.setVoltage(voltage);
	}

  @Override
	public void setShoulder(double percent) {
		leftShoulderMotor.set(percent);
		rightShoulderMotor.set(-percent);
	}

  @Override
	public void setElbow(double percent) {
		leftElbowMotor.set(percent);
		rightElbowMotor.set(-percent);
	}

  @Override
	public double getShoulderPosition() {
		// STOW: 0.601644465041112
		return shoulderAbsoluteEncoder.getDistance();
	}

  @Override
	public double getElbowPosition() {
		// STOW: 0.328055008201375
		return elbowAbsoluteEncoder.getDistance();
	}
  
  @Override
	public void setShoulderPosition(double position) {
		leftShoulderMotor.set(-shoulderPIDController.calculate(
				shoulderAbsoluteEncoder.getDistance() * 100, position * 100) * 10);
		rightShoulderMotor.set(shoulderPIDController.calculate(
				shoulderAbsoluteEncoder.getDistance() * 100, position * 100) * 10);

		SmartDashboard.putNumber("Shoulder Setter PID", shoulderPIDController.calculate(
				shoulderAbsoluteEncoder.getDistance() * 100, position * 100));

		SmartDashboard.putNumber("Shoulder Abs PID", shoulderAbsoluteEncoder.getDistance());
	}

  @Override
	public void setElbowPosition(double position) {
		leftElbowMotor.set(-elbowPIDController.calculate(
				elbowAbsoluteEncoder.getDistance() * 100, position * 100) * 10);
		rightElbowMotor.set(elbowPIDController.calculate(
				elbowAbsoluteEncoder.getDistance() * 100, position * 100) * 10);

		SmartDashboard.putNumber("Elbow Setter PID", elbowPIDController.calculate(
				elbowAbsoluteEncoder.getDistance() * 100, position * 100));

		SmartDashboard.putNumber("Elbow Abs PID", elbowAbsoluteEncoder.getDistance());
	}
  
}
