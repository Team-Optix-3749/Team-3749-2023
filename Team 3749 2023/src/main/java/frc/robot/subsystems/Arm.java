package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

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

	public Arm() {
		leftShoulderMotor.restoreFactoryDefaults();
		rightShoulderMotor.restoreFactoryDefaults();

		leftElbowMotor.restoreFactoryDefaults();
		rightElbowMotor.restoreFactoryDefaults();
	}

	public void setElbowVoltage(double voltage) {
		leftElbowMotor.setVoltage(voltage);
	}

	public void setShoulderVoltage(double voltage) {
		leftShoulderMotor.setVoltage(voltage);
	}

	public void setShoulder(double percent) {
		leftShoulderMotor.set(percent);
		rightShoulderMotor.set(-percent);
	}

	public void setElbow(double percent) {
		leftElbowMotor.set(percent);
		rightElbowMotor.set(-percent);
	}

	public double getShoulderPosition() {
		// STOW: 0.601644465041112
		return shoulderAbsoluteEncoder.getDistance();
	}

	public double getElbowPosition() {
		// STOW: 0.328055008201375
		return elbowAbsoluteEncoder.getDistance();
	}

	public void setShoulderPosition(double position) {
		leftShoulderMotor.set(-shoulderPIDController.calculate(
				shoulderAbsoluteEncoder.getDistance() * 100, position * 100) * 10);
		rightShoulderMotor.set(shoulderPIDController.calculate(
				shoulderAbsoluteEncoder.getDistance() * 100, position * 100) * 10);

		SmartDashboard.putNumber("Shoulder Setter PID", shoulderPIDController.calculate(
				shoulderAbsoluteEncoder.getDistance() * 100, position * 100));

		SmartDashboard.putNumber("Shoulder Abs PID", shoulderAbsoluteEncoder.getDistance());
	}

	public void setElbowPosition(double position) {
		leftElbowMotor.set(-elbowPIDController.calculate(
				elbowAbsoluteEncoder.getDistance() * 100, position * 100) * 10);
		rightElbowMotor.set(elbowPIDController.calculate(
				elbowAbsoluteEncoder.getDistance() * 100, position * 100) * 10);

		SmartDashboard.putNumber("Elbow Setter PID", elbowPIDController.calculate(
				elbowAbsoluteEncoder.getDistance() * 100, position * 100));

		SmartDashboard.putNumber("Elbow Abs PID", elbowAbsoluteEncoder.getDistance());
	}

	@Override
	public void simulationPeriodic() {
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elbow Abs", getElbowPosition());
		SmartDashboard.putNumber("Shoulder Abs", getShoulderPosition());
	}
}
