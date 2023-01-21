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
    private final ProfiledPIDController forearmController = new ProfiledPIDController(40.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(2, 5));
    private final ProfiledPIDController bicepController = new ProfiledPIDController(40.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(2, 5));
    private final Encoder forearmEncoder = new Encoder(0, 1);
    private final Encoder bicepEncoder = new Encoder(2, 3);

    private final CANSparkMax forearmMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax bicepMotor = new CANSparkMax(0 + 1, MotorType.kBrushless);

    // Simulation classes help us simulate what's going on, including gravity.
    private static final double armReduction = 600;
    private static final double forearmMass = 10.0; // Kilograms
    private static final double forearmLength = Units.inchesToMeters(38.5);
    private static final double bicepMass = 4.0; // Kilograms
    private static final double bicepLength = Units.inchesToMeters(27);

    private static final int forearm_min_angle = -75;
    private static final int forearm_max_angle = 260;
    private static final int bicep_min_angle = 30;
    private static final int bicep_max_angle = 150;

    // SETPOINTS FOR PRESETS MODE (Uses Virtual 4 Bar Mode for smooth movement)
    private static final int stowedBicep = 90;
    private static final int stowedForearm = 260;

    private static final int intakeBicep = 135;
    private static final int intakeForearm = 265;

    private static final int doubleSubstationBicep = 60;
    private static final int doubleSubstationForearm = 185;

    private static final int scoreFloorBicep = 120;
    private static final int scoreFloorForearm = 255;

    private static final int scoreMidBicep = 95;
    private static final int scoreMidForearm = 195;

    private static final int scoreHighBicep = 135;
    private static final int scoreHighForearm = 160;

    // This arm sim represents an arm that can travel from -75 degrees (rotated down
    // front)
    // to 255 degrees (rotated down in the back).
    private final SingleJointedArmSim forearmSim = new SingleJointedArmSim(
            armGearbox,
            armReduction,
            SingleJointedArmSim.estimateMOI(forearmLength, forearmMass),
            forearmLength,
            Units.degreesToRadians(forearm_min_angle),
            Units.degreesToRadians(forearm_max_angle),
            forearmMass,
            false,
            VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );
    private final SingleJointedArmSim bicepSim = new SingleJointedArmSim(
            armGearbox,
            armReduction,
            SingleJointedArmSim.estimateMOI(bicepLength, bicepMass),
            bicepLength,
            Units.degreesToRadians(bicep_min_angle),
            Units.degreesToRadians(bicep_max_angle),
            bicepMass,
            true,
            VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );
    private final EncoderSim forearmEncoderSim = new EncoderSim(forearmEncoder);
    private final EncoderSim bicepEncoderSim = new EncoderSim(bicepEncoder);
    SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
    SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.

    private final Mechanism2d mech2d = new Mechanism2d(90, 90);

    @Override
    public void simulationPeriodic() {
        forearmEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
        bicepEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
        SmartDashboard.putNumber("Setpoint forearm (degrees)", 90);
        SmartDashboard.putNumber("Setpoint bicep (degrees)", 90);
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
        forearmSim.setInput(forearmMotor.get() * RobotController.getBatteryVoltage());
        bicepSim.setInput(bicepMotor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        forearmSim.update(0.020);
        bicepSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        forearmEncoderSim.setDistance(forearmSim.getAngleRads());
        bicepEncoderSim.setDistance(bicepSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        forearmSim.getCurrentDrawAmps() + bicepSim.getCurrentDrawAmps()));
    }

}
