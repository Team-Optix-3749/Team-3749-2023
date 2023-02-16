package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.utils.Constants;

/**
 * Double jointed arm simulation adapted from
 * https://github.com/WHS-FRC-3467/DoubleJointedArmSim
 * 
 * @author Rohin Sood
 */
public class ArmSim extends Arm {

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  // = (2 * PI rads) / (4096 pulses)
  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor shoulderGearBox = DCMotor.getNEO(2);
  private final DCMotor elbowGearBox = DCMotor.getNEO(2);

  // Standard classes for controlling our arm
  private final Encoder elbowEncoder = new Encoder(0, 1);
  private final Encoder shoulderEncoder = new Encoder(2, 3);

  private final CANSparkMax leftElbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
  private final CANSparkMax rightElbowMotor = new CANSparkMax(Constants.Arm.right_elbow_id, MotorType.kBrushless);

  private final CANSparkMax leftShoulderMotor = new CANSparkMax(Constants.Arm.left_shoulder_id, MotorType.kBrushless);
  private final CANSparkMax rightShoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);

  private final double elbowMOI = SingleJointedArmSim.estimateMOI(Constants.Arm.elbow_reduction, Constants.Arm.forearm_length);
  private final LinearSystem<N2, N1, N1> elbowPlant = LinearSystemId.createSingleJointedArmSystem(
                elbowGearBox, elbowMOI, Constants.Arm.elbow_reduction);

  private final double shoulderMOI = SingleJointedArmSim.estimateMOI(Constants.Arm.shoulder_reduction, Constants.Arm.bicep_length);
  private final LinearSystem<N2, N1, N1> shoulderPlant = LinearSystemId.createSingleJointedArmSystem(
                shoulderGearBox, shoulderMOI, Constants.Arm.shoulder_reduction);


  // This arm sim represents an arm that can travel from -75 degrees (rotated down
  // front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim elbowSim = new SingleJointedArmSim(
      elbowPlant,
      elbowGearBox,
      Constants.Arm.elbow_reduction,
      Constants.Arm.forearm_length,
      Units.degreesToRadians(-360),
      Units.degreesToRadians(360),
      false,
      VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );
  private final SingleJointedArmSim shoulderSim = new SingleJointedArmSim(
      shoulderPlant,
      shoulderGearBox,
      Constants.Arm.shoulder_reduction,
      Constants.Arm.bicep_length,
      Units.degreesToRadians(-360),
      Units.degreesToRadians(360),
      false,
      VecBuilder.fill(kArmEncoderDistPerPulse) 
  );
  private final EncoderSim elbowEncoderSim = new EncoderSim(elbowEncoder);
  private final EncoderSim shoulderEncoderSim = new EncoderSim(shoulderEncoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mech2d = new Mechanism2d(90, 90);
  private final MechanismRoot2d midNodeHome = mech2d.getRoot("Mid Node", 27.83, 0);
  private final MechanismLigament2d midNode = midNodeHome
      .append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d highNodeHome = mech2d.getRoot("High Node", 10.58, 0);
  private final MechanismLigament2d highNode = highNodeHome
      .append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d gridHome = mech2d.getRoot("Grid Home", 49.75, 0);
  private final MechanismLigament2d gridNode = gridHome
      .append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d dsHome = mech2d.getRoot("Double Substation Home", 49.75, 37);
  private final MechanismLigament2d dsRamp = dsHome
      .append(new MechanismLigament2d("Double Substation Ramp", 13.75, 180, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d armPivot = mech2d.getRoot("Arm Pivot", 49.75 + 11, 21.75);
  private final MechanismLigament2d bicep = armPivot.append(
      new MechanismLigament2d(
          "Bicep",
          Constants.Arm.bicep_length,
          -90,
          10,
          new Color8Bit(Color.kGold)));
  private final MechanismLigament2d armTower = armPivot
      .append(new MechanismLigament2d("Arm Tower", 19.75, -90, 10, new Color8Bit(Color.kSilver)));

  private final MechanismLigament2d armSupport = armPivot
      .append(new MechanismLigament2d("Arm Support", 22.8, -60, 10, new Color8Bit(Color.kSilver)));
  private final MechanismLigament2d bumper = gridHome
      .append(new MechanismLigament2d("Bumper", 24, 0, 60, new Color8Bit(Color.kRed)));
  private final MechanismLigament2d forearm = bicep.append(
      new MechanismLigament2d(
          "Elbow",
          Constants.Arm.forearm_length,
          0,
          10,
          new Color8Bit(Color.kPurple)));
  private final MechanismLigament2d claw = forearm.append(
      new MechanismLigament2d(
          "Claw",
          Constants.Arm.claw_length,
          0,
          16,
          new Color8Bit(Color.kWhite)));

  public ArmSim() {
    rightElbowMotor.follow(leftElbowMotor);
    rightShoulderMotor.follow(leftShoulderMotor);

    rightElbowMotor.setInverted(true);
    rightShoulderMotor.setInverted(true);

    elbowEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
    shoulderEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", mech2d);
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
  public double getShoulderDistance() {
    return shoulderEncoder.getDistance();
  }

  @Override
  public double getElbowDistance() {
    return elbowEncoder.getDistance();
  }

  public void updateSim() {
    REVPhysicsSim.getInstance().run();

    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    elbowSim.setInput(leftElbowMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    shoulderSim.setInput(leftShoulderMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
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
    forearm.setAngle(Units.radiansToDegrees(elbowSim.getAngleRads()));
    bicep.setAngle(Units.radiansToDegrees(shoulderSim.getAngleRads()));
  }

  @Override
  public void simulationPeriodic() {
    updateSim();
  }

}