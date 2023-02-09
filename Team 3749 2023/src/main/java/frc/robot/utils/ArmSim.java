// package frc.robot.utils;

// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;

// public class ArmSim {

//   // Standard classes for controlling our arm
// 	public static Encoder elbowEncoder = new Encoder(0, 1);
// 	public static Encoder shoulderEncoder = new Encoder(2, 3);

//   // The arm gearbox represents a gearbox containing two Vex 775pro motors.
//   private static DCMotor shoulderGearBox = DCMotor.getNEO(2);
//   private static DCMotor elbowGearBox = DCMotor.getNEO(2);

//   public static SingleJointedArmSim elbowSim = new SingleJointedArmSim(
//       elbowGearBox,
//       Constants.Arm.elbow_reduction,
//       SingleJointedArmSim.estimateMOI(Constants.Arm.forearm_length, Constants.Arm.forearm_mass),
//       Constants.Arm.forearm_length,
//       Units.degreesToRadians(Constants.Arm.elbow_min_angle),
//       Units.degreesToRadians(Constants.Arm.elbow_max_angle),
//       Constants.Arm.forearm_mass,
//       false,
//       VecBuilder.fill(Constants.Arm.sim_encoder_dist_per_pulse) // Add noise with a std-dev of 1 tick
//   );
//   public static SingleJointedArmSim shoulderSim = new SingleJointedArmSim(
//       shoulderGearBox,
//       Constants.Arm.shoulder_reduction,
//       SingleJointedArmSim.estimateMOI(Constants.Arm.bicep_length, Constants.Arm.bicep_mass),
//       Constants.Arm.bicep_length,
//       Units.degreesToRadians(Constants.Arm.shoulder_min_angle),
//       Units.degreesToRadians(Constants.Arm.shoulder_max_angle),
//       Constants.Arm.bicep_mass,
//       false,
//       VecBuilder.fill(Constants.Arm.sim_encoder_dist_per_pulse) // Add noise with a std-dev of 1 tick
//   );
//   public static EncoderSim elbowEncoderSim = new EncoderSim(elbowEncoder);
//   public static EncoderSim shoulderEncoderSim = new EncoderSim(shoulderEncoder);
//   public static SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

//   // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
//   public static Mechanism2d mech2d = new Mechanism2d(90, 90);
//   public static MechanismRoot2d midNodeHome = mech2d.getRoot("Mid Node", 33.5, 0);
//   public static MechanismLigament2d midNode = midNodeHome
//       .append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
//   public static MechanismRoot2d highNodeHome = mech2d.getRoot("High Node", 16.5, 0);
//   public static MechanismLigament2d highNode = highNodeHome
//       .append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
//   public static MechanismRoot2d gridHome = mech2d.getRoot("Grid Home", 49.75, 0);
//   public static MechanismLigament2d gridNode = gridHome
//       .append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));
//   public static MechanismRoot2d dsHome = mech2d.getRoot("Double Substation Home", 49.75, 36.75);
//   public static MechanismLigament2d dsRamp = dsHome
//       .append(new MechanismLigament2d("Double Substation Ramp", 13.75, 180, 10, new Color8Bit(Color.kWhite)));
//   public static MechanismRoot2d armPivot = mech2d.getRoot("Arm Pivot", 49.75 + 11, 19.75);
//   public static MechanismLigament2d bicep = armPivot.append(
//       new MechanismLigament2d(
//           "Bicep",
//           Constants.Arm.bicep_length,
//           30,
//           10,
//           new Color8Bit(Color.kGold)));
//   public static MechanismLigament2d armTower = armPivot
//       .append(new MechanismLigament2d("Arm Tower", 19.75, -90, 10, new Color8Bit(Color.kSilver)));
//   public static MechanismLigament2d armSupport = armPivot
//       .append(new MechanismLigament2d("Arm Support", 30, -60, 10, new Color8Bit(Color.kSilver)));
//   public static MechanismLigament2d bumper = gridHome
//       .append(new MechanismLigament2d("Bumper", 24, 0, 60, new Color8Bit(Color.kRed)));
//   public static MechanismLigament2d forearm = bicep.append(
//       new MechanismLigament2d(
//           "Elbow",
//           Constants.Arm.forearm_length,
//           Units.radiansToDegrees(elbowSim.getAngleRads()),
//           10,
//           new Color8Bit(Color.kPurple)));
//   public static MechanismLigament2d claw = forearm.append(
//       new MechanismLigament2d(
//           "Claw",
//           Constants.Arm.claw_length,
//           Units.radiansToDegrees(elbowSim.getAngleRads()),
//           16,
//           new Color8Bit(Color.kWhite)));

//   public ArmSim() {
//     elbowEncoder.setDistancePerPulse(Constants.Arm.sim_encoder_dist_per_pulse);
// 		shoulderEncoder.setDistancePerPulse(Constants.Arm.sim_encoder_dist_per_pulse);

//     presetChooser.setDefaultOption("Starting Position", 0);
//     presetChooser.addOption("Floor Intake Position", 1);
//     presetChooser.addOption("Double Substation Intake", 2);
//     presetChooser.addOption("Floor Node Score", 3);
//     presetChooser.addOption("Mid Node Score", 4);
//     presetChooser.addOption("High Node Score", 5);
//     SmartDashboard.putData(presetChooser);
//     // Put Mechanism 2d to SmartDashboard
//     SmartDashboard.putData("Arm Sim", mech2d);
//   }
// }
