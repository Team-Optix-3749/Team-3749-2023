package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSWDLJNI;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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

public class ArmSim extends SubsystemBase {

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 42;

    // The arm gearbox represents a gearbox containing two neo brushless motors.
    private final DCMotor armGearbox = DCMotor.getNEO(2);

    // Standard classes for controlling our arm
    private final ProfiledPIDController elbowController = new ProfiledPIDController(40.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(2, 5));
    private final ProfiledPIDController shoulderController = new ProfiledPIDController(40.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(2, 5));
    private final Encoder elbowEncoder = new Encoder(0, 1);
    private final Encoder shoulderEncoder = new Encoder(2, 3);

    private final CANSparkMax leftelbowMotor = new CANSparkMax(15, MotorType.kBrushless);
    private final CANSparkMax rightelbowMotor = new CANSparkMax(16, MotorType.kBrushless);

    private final CANSparkMax leftshoulderMotor = new CANSparkMax(17, MotorType.kBrushless);
    private final CANSparkMax rightshoulderMotor = new CANSparkMax(18 + 1, MotorType.kBrushless);


    // Simulation classes help us simulate what's going on, including gravity.
    private static final double armReduction = 600;
    private static final double elbowMass = 10.0; // Kilograms
    private static final double elbowLength = Units.inchesToMeters(38.5);
    private static final double shoulderMass = 4.0; // Kilograms
    private static final double shoulderLength = Units.inchesToMeters(27);

    private static final int elbow_min_angle = -75;
    private static final int elbow_max_angle = 260;
    private static final int shoulder_min_angle = 30;
    private static final int shoulder_max_angle = 150;

    // SETPOINTS FOR PRESETS MODE (Uses Virtual 4 Bar Mode for smooth movement)
    private static final int stowedshoulder = 90;
    private static final int stowedelbow = 260;

    private static final int intakeshoulder = 135;
    private static final int intakeelbow = 265;

    private static final int doubleSubstationshoulder = 60;
    private static final int doubleSubstationelbow = 185;

    private static final int scoreFloorshoulder = 120;
    private static final int scoreFloorelbow = 255;

    private static final int scoreMidshoulder = 95;
    private static final int scoreMidelbow = 195;

    private static final int scoreHighshoulder = 135;
    private static final int scoreHighelbow = 160;

    // This arm sim represents an arm that can travel from -75 degrees (rotated down
    // front)
    // to 255 degrees (rotated down in the back).
    private final SingleJointedArmSim elbowSim = new SingleJointedArmSim(
            armGearbox,
            armReduction,
            SingleJointedArmSim.estimateMOI(elbowLength, elbowMass),
            elbowLength,
            Units.degreesToRadians(elbow_min_angle),
            Units.degreesToRadians(elbow_max_angle),
            elbowMass,
            false,
            VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );
    private final SingleJointedArmSim shoulderSim = new SingleJointedArmSim(
            armGearbox,
            armReduction,
            SingleJointedArmSim.estimateMOI(shoulderLength, shoulderMass),
            shoulderLength,
            Units.degreesToRadians(shoulder_min_angle),
            Units.degreesToRadians(shoulder_max_angle),
            shoulderMass,
            true,
            VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );
    private final EncoderSim elbowEncoderSim = new EncoderSim(elbowEncoder);
    private final EncoderSim shoulderEncoderSim = new EncoderSim(shoulderEncoder);
    SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
    SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.

    private final Mechanism2d mech2d = new Mechanism2d(90, 90);

    @Override
    public void simulationPeriodic() {
        elbowEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
        shoulderEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
        SmartDashboard.putNumber("Setpoint elbow (degrees)", 90);
        SmartDashboard.putNumber("Setpoint shoulder (degrees)", 90);
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
        SmartDashboard.putData("Arm Sim", mech2d);

        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        elbowSim.setInput(leftelbowMotor.get() * RobotController.getBatteryVoltage());
        shoulderSim.setInput(leftshoulderMotor.get() * RobotController.getBatteryVoltage());

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
    }

}
