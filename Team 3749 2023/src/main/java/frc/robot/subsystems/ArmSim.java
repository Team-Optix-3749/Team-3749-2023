package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
import frc.robot.utils.Constants;
import frc.robot.utils.SmartData;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSim extends SubsystemBase {

	private static final int kMotorPort = 0;
	private static final int kEncoderAChannel = 0;
	private static final int kEncoderBChannel = 1;
	private static final int kJoystickPort = 0;

	// The P gain for the PID controller that drives this arm.
	private static final double kArmKp = 1.0;
	private static final double kArmKi = 0.0;

	// distance per pulse = (angle per revolution) / (pulses per revolution)
	// = (2 * PI rads) / (4096 pulses)
	private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

	// The arm gearbox represents a gearbox containing two Vex 775pro motors.
	private final DCMotor m_armGearbox = DCMotor.getNEO(2);

	// Standard classes for controlling our arm
	public final Encoder m_topEncoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
	public final Encoder m_bottomEncoder = new Encoder(kEncoderAChannel + 2, kEncoderBChannel + 2);

	public final CANSparkMax m_topMotor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
	public final CANSparkMax m_bottomMotor = new CANSparkMax(kMotorPort + 1, MotorType.kBrushless);

	// Simulation classes help us simulate what's going on, including gravity.
	private static final double m_armReduction = 600;
	private static final double m_arm_topMass = 10.0; // Kilograms
	private static final double m_arm_topLength = Units.inchesToMeters(38.5);
	private static final double m_arm_bottomMass = 4.0; // Kilograms
	private static final double m_arm_bottomLength = Units.inchesToMeters(27);

	public static final int m_arm_top_min_angle = -75;
	public static final int m_arm_top_max_angle = 260;
	public static final int m_arm_bottom_min_angle = 30;
	public static final int m_arm_bottom_max_angle = 150;

	public final ProfiledPIDController m_topController = new ProfiledPIDController(kArmKp, kArmKi, 0,
			new TrapezoidProfile.Constraints(2, 5));
	public final ProfiledPIDController m_bottomController = new ProfiledPIDController(kArmKp, kArmKi, 0,
			new TrapezoidProfile.Constraints(2, 5));

	// This arm sim represents an arm that can travel from -75 degrees (rotated down
	// front)
	// to 255 degrees (rotated down in the back).
	private final SingleJointedArmSim m_arm_topSim = new SingleJointedArmSim(
			m_armGearbox,
			m_armReduction,
			SingleJointedArmSim.estimateMOI(m_arm_topLength, m_arm_topMass),
			m_arm_topLength,
			Units.degreesToRadians(m_arm_top_min_angle),
			Units.degreesToRadians(m_arm_top_max_angle),
			m_arm_topMass,
			false,
			VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
	);
	private final SingleJointedArmSim m_arm_bottomSim = new SingleJointedArmSim(
			m_armGearbox,
			m_armReduction,
			SingleJointedArmSim.estimateMOI(m_arm_bottomLength, m_arm_bottomMass),
			m_arm_bottomLength,
			Units.degreesToRadians(m_arm_bottom_min_angle),
			Units.degreesToRadians(m_arm_bottom_max_angle),
			m_arm_bottomMass,
			true,
			VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
	);
	private final EncoderSim m_topEncoderSim = new EncoderSim(m_topEncoder);
	private final EncoderSim m_bottomEncoderSim = new EncoderSim(m_bottomEncoder);
	public SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
	public SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

	// Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
	private final Mechanism2d m_mech2d = new Mechanism2d(90, 90);
	private final MechanismRoot2d midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0);
	private final MechanismLigament2d MidNode = midNodeHome
			.append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
	private final MechanismRoot2d highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0);
	private final MechanismLigament2d HighNode = highNodeHome
			.append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
	private final MechanismRoot2d gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0);
	private final MechanismLigament2d GridNode = gridHome
			.append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));
	private final MechanismRoot2d dsHome = m_mech2d.getRoot("Double Substation Home", 49.75, 37);
	private final MechanismLigament2d DSRamp = dsHome
			.append(new MechanismLigament2d("Double Substation Ramp", 13.75, 180, 10, new Color8Bit(Color.kWhite)));
	private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 65, 21.75);
	private final MechanismLigament2d m_arm_bottom = m_armPivot.append(
			new MechanismLigament2d(
					"Arm Bottom",
					27,
					-90,
					10,
					new Color8Bit(Color.kGold)));
	private final MechanismLigament2d m_arm_tower = m_armPivot
			.append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kSilver)));

	private final MechanismLigament2d m_aframe_1 = m_armPivot
			.append(new MechanismLigament2d("aframe1", 24, -50, 10, new Color8Bit(Color.kSilver)));
	private final MechanismLigament2d m_bumper = gridHome
			.append(new MechanismLigament2d("Bumper", 30.5, 0, 60, new Color8Bit(Color.kRed)));
	private final MechanismLigament2d m_arm_top = m_arm_bottom.append(
			new MechanismLigament2d(
					"Arm Top",
					28.5 + 3.0,
					Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
					10,
					new Color8Bit(Color.kPurple)));
	private final MechanismLigament2d m_intake = m_arm_top.append(
			new MechanismLigament2d(
					"Intake",
					7,
					Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
					40,
					new Color8Bit(Color.kWhite)));

	public ArmSim() {
		m_topEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
		m_bottomEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);

		// topRelativeEncoder.setPositionConversionFactor(kArmEncoderDistPerPulse / 42);
		// bottomRelativeEncoder.setPositionConversionFactor(kArmEncoderDistPerPulse /
		// 42);

		SmartDashboard.putNumber("Setpoint top (degrees)", 90);
		SmartDashboard.putNumber("Setpoint bottom (degrees)", 90);
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
		// Put Mechanism 2d to SmartDashboard
		SmartDashboard.putData("Arm Sim", m_mech2d);
	}

	public void updateSim() {
		// In this method, we update our simulation of what our arm is doing
		// First, we set our "inputs" (voltages)
		m_arm_topSim.setInput(m_topMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
		m_arm_bottomSim.setInput(m_bottomMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
		// Next, we update it. The standard loop time is 20ms.
		m_arm_topSim.update(0.020);
		m_arm_bottomSim.update(0.020);

		// Finally, we set our simulated encoder's readings and simulated battery
		// voltage
		m_topEncoderSim.setDistance(m_arm_topSim.getAngleRads());
		m_bottomEncoderSim.setDistance(m_arm_bottomSim.getAngleRads());
		// SimBattery estimates loaded battery voltages
		RoboRioSim.setVInVoltage(
				BatterySim.calculateDefaultBatteryLoadedVoltage(
						m_arm_topSim.getCurrentDrawAmps() + m_arm_bottomSim.getCurrentDrawAmps()));

		// Update the Mechanism Arm angle based on the simulated arm angle
		m_arm_top.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()));
		m_arm_bottom.setAngle(Units.radiansToDegrees(m_arm_bottomSim.getAngleRads()));
	}

	@Override
	public void simulationPeriodic() {
		updateSim();
	}

}
