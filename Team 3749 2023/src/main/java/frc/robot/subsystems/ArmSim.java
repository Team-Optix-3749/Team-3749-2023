package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.SmartData;

public class ArmSim extends SubsystemBase {

	// distance per pulse = (angle per revolution) / (pulses per revolution)
	// = (2 * PI rads) / (4096 pulses)
	private static final double encoder_dist_per_pulse = (2.0 * Math.PI) / 4096;

	// Standard classes for controlling our arm
	private final ProfiledPIDController elbowController = new ProfiledPIDController(40.0, 0.0, 0.0,
			new TrapezoidProfile.Constraints(2, 5));
	private final ProfiledPIDController shoulderController = new ProfiledPIDController(40.0, 0.0, 0.0,
			new TrapezoidProfile.Constraints(2, 5));

	private final CANSparkMax leftElbowMotor = new CANSparkMax(15, MotorType.kBrushless);
	private final CANSparkMax rightElbowMotor = new CANSparkMax(16, MotorType.kBrushless);

	private final CANSparkMax leftShoulderMotor = new CANSparkMax(17, MotorType.kBrushless);
	private final CANSparkMax rightShoulderMotor = new CANSparkMax(18, MotorType.kBrushless);

	private final Encoder shoulderEncoder = new Encoder(0, 1);
	private final Encoder elbowEncoder = new Encoder(4, 5);

	private final EncoderSim shoulderEncoderSim = new EncoderSim(shoulderEncoder);
	private final EncoderSim elbowEncoderSim = new EncoderSim(elbowEncoder);

	public SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
	public SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

	// This arm sim represents an arm that can travel from -75 degrees (rotated down
	// front)
	// to 255 degrees (rotated down in the back).
	private final SingleJointedArmSim elbowSim = new SingleJointedArmSim(
			DCMotor.getNEO(2),
			Constants.Arm.elbow_reduction,
			SingleJointedArmSim.estimateMOI(Constants.Arm.forearm_length, Constants.Arm.forearm_mass),
			Constants.Arm.forearm_length,
			Units.degreesToRadians(Constants.Arm.elbow_min_angle),
			Units.degreesToRadians(Constants.Arm.elbow_max_angle),
			Constants.Arm.forearm_mass,
			true,
			VecBuilder.fill(encoder_dist_per_pulse) // Add noise with a std-dev of 1 tick
	);
	private final SingleJointedArmSim shoulderSim = new SingleJointedArmSim(
			DCMotor.getNEO(2),
			Constants.Arm.shoulder_reduction,
			SingleJointedArmSim.estimateMOI(Constants.Arm.bicep_length, Constants.Arm.bicep_mass),
			Constants.Arm.bicep_length,
			Units.degreesToRadians(Constants.Arm.shoulder_min_angle),
			Units.degreesToRadians(Constants.Arm.shoulder_max_angle),
			Constants.Arm.bicep_mass,
			true,
			VecBuilder.fill(encoder_dist_per_pulse) // Add noise with a std-dev of 1 tick
	);

	// Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
	private final Mechanism2d mech2d = new Mechanism2d(90, 90);

	private final MechanismRoot2d midNodeHome = mech2d.getRoot("mid node", 27.83, 0);
	private final MechanismLigament2d midNode = midNodeHome
			.append(new MechanismLigament2d("mid cone node", 34, 90, 10, new Color8Bit(Color.kWhite)));
	private final MechanismRoot2d highNodeHome = mech2d.getRoot("migh node", 10.58, 0);
	private final MechanismLigament2d highNode = highNodeHome
			.append(new MechanismLigament2d("high cone node", 46, 90, 10, new Color8Bit(Color.kWhite)));
	private final MechanismRoot2d gridHome = mech2d.getRoot("Grid Home", 49.75, 0);
	private final MechanismLigament2d gridNode = gridHome
			.append(new MechanismLigament2d("grid wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));
	private final MechanismRoot2d dsHome = mech2d.getRoot("Double Substation Home", 49.75, 37);	w
	private final MechanismLigament2d dsRamp = dsHome
			.append(new MechanismLigament2d("double substation ramp", 13.75, 180, 10, new Color8Bit(Color.kWhite)));
	private final MechanismRoot2d armRoot = mech2d.getRoot("arm root", 65, 21.75);
	private final MechanismLigament2d armTower = armRoot
			.append(new MechanismLigament2d("arm tower", 18, -90, 10, new Color8Bit(Color.kSilver)));
	private final MechanismLigament2d bicep = armRoot.append(
			new MechanismLigament2d(
					"bicep",
					Constants.Arm.bicep_length,
					Constants.Arm.shoulder_min_angle,
					10,
					new Color8Bit(Color.kGold)));

	private final MechanismLigament2d armSupport = armRoot
			.append(new MechanismLigament2d("arm support", 24, -50, 10, new Color8Bit(Color.kSilver)));
	private final MechanismLigament2d bumper = gridHome
			.append(new MechanismLigament2d("bumper", 30.5, 0, 60, new Color8Bit(Color.kRed)));
	private final MechanismLigament2d forearm = bicep.append(
			new MechanismLigament2d(
					"forearm",
					Constants.Arm.forearm_length,
					Units.radiansToDegrees(elbowSim.getAngleRads()),
					10,
					new Color8Bit(Color.kPurple)));
	private final MechanismLigament2d claw = forearm.append(
			new MechanismLigament2d(
					"claw",
					Constants.Arm.claw_length,
					Units.radiansToDegrees(elbowSim.getAngleRads()),
					20,
					new Color8Bit(Color.kWhite)));

	public ArmSim() {
		rightElbowMotor.setInverted(true);
		rightShoulderMotor.setInverted(true);

		// right motors are follower motors for left motors
		rightShoulderMotor.follow(leftShoulderMotor);
		rightElbowMotor.follow(leftElbowMotor);

		// Put Mechanism 2d to SmartDashboard
		SmartDashboard.putData("Arm Sim", mech2d);

		REVPhysicsSim.getInstance().addSparkMax(leftElbowMotor, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(rightElbowMotor, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(leftShoulderMotor, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(rightShoulderMotor, DCMotor.getNEO(1));

		elbowEncoder.setDistancePerPulse(encoder_dist_per_pulse);
		shoulderEncoder.setDistancePerPulse(encoder_dist_per_pulse);

		controlMode.setDefaultOption("Presets (Setpoints)", 0);
		controlMode.addOption("Virtual Four Bar", 1);
		controlMode.addOption("Manual Angle Adjust", 2);

		presetChooser.setDefaultOption("Starting Position", 0);
		presetChooser.addOption("Floor Intake Position", 1);
		presetChooser.addOption("Double Substation Intake", 2);
		presetChooser.addOption("Floor Node Score", 3);
		presetChooser.addOption("Mid Node Score", 4);
		presetChooser.addOption("High Node Score", 5);

		SmartDashboard.putData(controlMode);
		SmartDashboard.putData(presetChooser);
	}

	@Override
	public void simulationPeriodic() {
		updateSim();
	}

	public double getElbowEncoderDistance() {
		return elbowEncoder.getDistance();
	}

	public double getShoulderEncoderDistance() {
		return shoulderEncoder.getDistance();
	}

	public void setElbowVoltage(double voltage) {
		leftElbowMotor.setVoltage(voltage);
		rightElbowMotor.setVoltage(voltage);

		SmartDashboard.putNumber("elbow angle", elbowEncoder.getRaw());
	}

	public void setShoulderVoltage(double voltage) {
		leftShoulderMotor.setVoltage(voltage);
		rightShoulderMotor.setVoltage(voltage);

		SmartDashboard.putNumber("shoulder angle", shoulderEncoder.getRaw());
	}

	public void updateSim() {
		// In this method, we update our simulation of what our arm is doing
		// First, we set our "inputs" (voltages)
		elbowSim.setInputVoltage(leftElbowMotor.get() * RobotController.getBatteryVoltage());
		shoulderSim.setInputVoltage(leftShoulderMotor.get() * RobotController.getBatteryVoltage());

		SmartDashboard.putNumber("elbowSIm", Units.radiansToDegrees(elbowSim.getAngleRads()));
		SmartDashboard.putNumber("shoulderSim", Units.radiansToDegrees(shoulderSim.getAngleRads()));

		SmartDashboard.putNumber("elbow angle", Units.radiansToDegrees(elbowSim.getAngleRads()));

		// Next, we update it. The standard loop time is 20ms.
		elbowSim.update(0.020);
		shoulderSim.update(0.020);

		// Finally, we set our simulated encoder's readings and simulated battery
		// voltage
		elbowEncoderSim.setDistance(elbowSim.getAngleRads());	
		shoulderEncoderSim.setDistance(shoulderSim.getAngleRads());
		// SimBattery estimates loaded battery voltages
		RoboRioSim.setVInVoltage(
				BatterySim.calculateDefaultBatteryLoadedVoltage(
						elbowSim.getCurrentDrawAmps() + shoulderSim.getCurrentDrawAmps()));

		// Update the Mechanism Arm angle based on the simulated arm angle
		bicep.setAngle(Units.radiansToDegrees(shoulderSim.getAngleRads()));
		forearm.setAngle(Units.radiansToDegrees(elbowSim.getAngleRads()));
	}

}
