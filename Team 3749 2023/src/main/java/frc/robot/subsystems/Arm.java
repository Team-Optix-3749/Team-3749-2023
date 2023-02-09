package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.utils.ArmSim;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

	private final CANSparkMax leftElbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
	private final CANSparkMax rightElbowMotor = new CANSparkMax(Constants.Arm.right_elbow_id, MotorType.kBrushless);
	private final AbsoluteEncoder elbowAbsoluteEncoder = leftElbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

	private final CANSparkMax leftShoulderMotor = new CANSparkMax(Constants.Arm.left_shoulder_id, MotorType.kBrushless);
	private final CANSparkMax rightShoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);
	private final AbsoluteEncoder shoulderAbsoluteEncoder = leftShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);

	// This arm sim represents an arm that can travel from -75 degrees (rotated down
	// front)
	// to 255 degrees (rotated down in the back).

	public Arm() {
		rightElbowMotor.follow(leftElbowMotor);
		// rightShoulderMotor.follow(leftShoulderMotor);

		rightElbowMotor.setInverted(true);
		rightShoulderMotor.setInverted(true);

		// SmartDashboard.putNumber("Setpoint top (degrees)", 90);
		// SmartDashboard.putNumber("Setpoint bottom (degrees)", 90);

		// Constants.Arm.controlMode.setDefaultOption("Joystick Control", 0);
		// Constants.Arm.controlMode.addOption("Presets", 1);
		// SmartDashboard.putData(Constants.Arm.controlMode);

	// 	ArmSim.elbowEncoder.setDistancePerPulse(Constants.Arm.sim_encoder_dist_per_pulse);
	// 	ArmSim.shoulderEncoder.setDistancePerPulse(Constants.Arm.sim_encoder_dist_per_pulse);

  //   ArmSim.presetChooser.setDefaultOption("Starting Position", 0);
  //   ArmSim.presetChooser.addOption("Floor Intake Position", 1);
  //   ArmSim.presetChooser.addOption("High Node Score", 5);
  //   SmartDashboard.putData(ArmSim.presetChooser);
  //   // Put Mechanism 2d to SmartDashboard
  //   SmartDashboard.putData("Arm Sim", ArmSim.mech2d);
	}
	
	public void setElbowVoltage(double voltage) {
		leftElbowMotor.setVoltage(voltage);
	}

	public void setElbowPosition(double position) {

	}

	public void setShoulderPosition(double position) {
	}

	public void setShoulderVoltage(double voltage) {
		leftShoulderMotor.setVoltage(voltage);
	}

	public void setShoulder(double percent) {
		System.out.println("a;lsdkjf;laskdghlkasdgh");
		leftShoulderMotor.set(percent);
		rightShoulderMotor.set(percent);
	}

	public double getElbowPosition() {
		return elbowAbsoluteEncoder.getPosition();
	}

	public double getShoulderPosition() {
		return shoulderAbsoluteEncoder.getPosition();
	}

	@Override
	public void simulationPeriodic() {
		// REVPhysicsSim.getInstance().run();

		// In this method, we update our simulation of what our arm is doing
		// First, we set our "inputs" (voltages)
		// ArmSim.elbowSim.setInput(leftElbowMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
		// ArmSim.shoulderSim.setInput(leftShoulderMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
		// // Next, we update it. The standard loop time is 20ms.
		// ArmSim.elbowSim.update(0.020);
		// ArmSim.shoulderSim.update(0.020);

		// // Finally, we set our simulated encoder's readings and simulated battery
		// // voltage
		// ArmSim.elbowEncoderSim.setDistance(ArmSim.elbowSim.getAngleRads());
		// ArmSim.shoulderEncoderSim.setDistance(ArmSim.shoulderSim.getAngleRads());
		
		// // SimBattery estimates loaded battery voltages
		// RoboRioSim.setVInVoltage(
		// 		BatterySim.calculateDefaultBatteryLoadedVoltage(
		// 				ArmSim.elbowSim.getCurrentDrawAmps() + ArmSim.shoulderSim.getCurrentDrawAmps()));

		// // Update the Mechanism Arm angle based on the simulated arm angle
		// ArmSim.forearm.setAngle(Units.radiansToDegrees(ArmSim.elbowSim.getAngleRads()));
		// ArmSim.bicep.setAngle(Units.radiansToDegrees(ArmSim.shoulderSim.getAngleRads()));
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elbow Abs", getElbowPosition());
		SmartDashboard.putNumber("Shoulder Abs", getShoulderPosition());

		SmartDashboard.putNumber("left elbow voltage", leftElbowMotor.getAppliedOutput() * leftElbowMotor.getBusVoltage());
		SmartDashboard.putNumber("left shoulder voltage", leftShoulderMotor.getAppliedOutput() * leftShoulderMotor.getBusVoltage());
		SmartDashboard.putNumber("right elbow voltage", rightElbowMotor.getAppliedOutput() * rightElbowMotor.getBusVoltage());
		SmartDashboard.putNumber("right shoulder voltage", rightShoulderMotor.getAppliedOutput() * rightShoulderMotor.getBusVoltage());
	}

}
