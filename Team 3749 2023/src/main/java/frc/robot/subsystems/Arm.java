package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
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
import frc.robot.utils.Constants;

/*
 * @author Bailey Say
 * 
 * Code for the arm of the robot. This does not include the claw at the end
 */

public class Arm extends SubsystemBase {
    // Motors, PIDS, Encoders, and other variables & instantiations 
    private CANSparkMax lowerNeoMotor = new CANSparkMax(Constants.Arm.neo_motor_lower_port, MotorType.kBrushless); // Check if this is actually brushless later
    private CANSparkMax upperNeoMotor = new CANSparkMax(Constants.Arm.neo_motor_upper_port, MotorType.kBrushless); // Check if this is actually brushless later
    
    // level (temporary might change later)
    private int level = 0;

    private final DCMotor armGearbox = DCMotor.getNEO(Constants.Arm.number_of_motors);
        
    // Standard classes for controlling our arm
    private final ProfiledPIDController topController = new ProfiledPIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd, new TrapezoidProfile.Constraints(Constants.Arm.max_velocity, Constants.Arm.max_acceleration));
    private final ProfiledPIDController bottomController = new ProfiledPIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd, new TrapezoidProfile.Constraints(Constants.Arm.max_velocity, Constants.Arm.max_acceleration));
    
    // Relative Encoders
    private final RelativeEncoder topEncoder = upperNeoMotor.getEncoder();
    private final RelativeEncoder bottomEncoder = lowerNeoMotor.getEncoder();

    // Simulation Code (gravity not accounted for)
    private final SingleJointedArmSim armTopSim = new SingleJointedArmSim(
            armGearbox, 
            Constants.Simulation.arm_reduction, 
            SingleJointedArmSim.estimateMOI(Constants.Simulation.arm_top_length, Constants.Simulation.arm_top_mass),
            Constants.Simulation.arm_top_length,
            Units.degreesToRadians(Constants.Simulation.arm_top_min_angle),
            Units.degreesToRadians(Constants.Simulation.arm_top_max_angle),
            Constants.Simulation.arm_top_mass,
            false, // gravity not accounted for
            VecBuilder.fill(Constants.Simulation.arm_encoder_dist_per_pulse)
            ); // Add noise with a std-dev of 1 tick);

    private final SingleJointedArmSim armBottomSim = new SingleJointedArmSim(armGearbox, 
            Constants.Simulation.arm_reduction, 
            SingleJointedArmSim.estimateMOI(Constants.Simulation.arm_bottom_length, Constants.Simulation.arm_bottom_mass),
            Constants.Simulation.arm_bottom_length,
            Units.degreesToRadians(Constants.Simulation.arm_bottom_min_angle),
            Units.degreesToRadians(Constants.Simulation.arm_bottom_max_angle),
            Constants.Simulation.arm_bottom_mass,
            false, // gravity not accounted for
            VecBuilder.fill(Constants.Simulation.arm_encoder_dist_per_pulse)
            ); // Add noise with a std-dev of 1 tick);
    
    // gui stuff lol
    // generic encoder, delete later
    Encoder upEncoder = new Encoder(0, 1);
    Encoder lowEncoder = new Encoder(2,3);

    private final EncoderSim topEncoderSim = new EncoderSim(upEncoder);
    private final EncoderSim bottomEncoderSim = new EncoderSim(lowEncoder);
    SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
    SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d m_mech2d = new Mechanism2d(90, 90);
    private final MechanismRoot2d midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0);
    private final MechanismLigament2d MidNode = midNodeHome.append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0);
    private final MechanismLigament2d HighNode = highNodeHome.append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0);
    private final MechanismLigament2d GridNode = gridHome.append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d dsHome = m_mech2d.getRoot("Double Substation Home", 49.75, 37);
    private final MechanismLigament2d DSRamp = dsHome.append(new MechanismLigament2d("Double Substation Ramp", 13.75, 180, 10, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 65, 21.75);
    private final MechanismLigament2d m_arm_bottom =
        m_armPivot.append(
            new MechanismLigament2d(
                "Arm Bottom",
                27, 
                -90, 
                10, 
                new Color8Bit(Color.kGold)));
    private final MechanismLigament2d armTower =
        m_armPivot.append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kSilver)));

    private final MechanismLigament2d aframe1 =
        m_armPivot.append(new MechanismLigament2d("aframe1", 24, -50, 10, new Color8Bit(Color.kSilver)));
    private final MechanismLigament2d bumper =
        gridHome.append(new MechanismLigament2d("Bumper", 30.5, 0, 60, new Color8Bit(Color.kRed)));
    private final MechanismLigament2d m_arm_top =
        m_arm_bottom.append(
            new MechanismLigament2d(
                "Arm Top",
                28.5 + 3.0,
                Units.radiansToDegrees(armTopSim.getAngleRads()),
                10,
                new Color8Bit(Color.kPurple)));
    private final MechanismLigament2d intake =
    m_arm_top.append(
        new MechanismLigament2d(
            "Intake",
            7,
            Units.radiansToDegrees(armTopSim.getAngleRads()),
            40,
            new Color8Bit(Color.kWhite)));
    
    
    public void setUpSim(){
        (upEncoder).setDistancePerPulse(Constants.Simulation.arm_encoder_dist_per_pulse);
        (lowEncoder).setDistancePerPulse(Constants.Simulation.arm_encoder_dist_per_pulse);
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
            
    public void periodicSim(){
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        armTopSim.setInput(upperNeoMotor.get() * RobotController.getBatteryVoltage());
        armBottomSim.setInput(lowerNeoMotor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        armTopSim.update(0.020);
        armBottomSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        topEncoderSim.setDistance(armTopSim.getAngleRads());
        bottomEncoderSim.setDistance(armBottomSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(armTopSim.getCurrentDrawAmps() + armBottomSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        m_arm_top.setAngle(Units.radiansToDegrees(armTopSim.getAngleRads()));
        m_arm_bottom.setAngle(Units.radiansToDegrees(armBottomSim.getAngleRads()));
    }
    
    public void teleopPeriodicSim(){
        switch(controlMode.getSelected()){
            case 1:
                // Here, we run PID control where the top arm acts like a four-bar relative to the bottom. 
                double pidOutputTop = topController.calculate(upEncoder.getDistance(), Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint top (degrees)", 0) - MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 150), Constants.Simulation.arm_bottom_min_angle, Constants.Simulation.arm_bottom_max_angle), Constants.Simulation.arm_top_min_angle, Constants.Simulation.arm_top_max_angle)));
                upperNeoMotor.setVoltage(pidOutputTop);
            
                double pidOutputBottom = bottomController.calculate(lowEncoder.getDistance(), Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0), Constants.Simulation.arm_bottom_min_angle, Constants.Simulation.arm_bottom_max_angle)));
                lowerNeoMotor.setVoltage(pidOutputBottom);
                break;
            case 2:
                pidOutputTop = topController.calculate(upEncoder.getDistance(), Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint top (degrees)", 0), Constants.Simulation.arm_top_min_angle, Constants.Simulation.arm_top_max_angle)));
                upperNeoMotor.setVoltage(pidOutputTop);
        
                pidOutputBottom = bottomController.calculate(lowEncoder.getDistance(), Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0), Constants.Simulation.arm_bottom_min_angle, Constants.Simulation.arm_bottom_max_angle)));
                lowerNeoMotor.setVoltage(pidOutputBottom);
                break;
            default: //also case 0
                int topSetpoint, bottomSetpoint;
                switch(presetChooser.getSelected()){
                case 0:
                    topSetpoint = Constants.Simulation.stowed_top;
                    bottomSetpoint = Constants.Simulation.stowed_bottom;
                    break;
                case 1:
                    topSetpoint = Constants.Simulation.intake_top;
                    bottomSetpoint = Constants.Simulation.intake_bottom;
                    break;
                case 2:
                    topSetpoint = Constants.Simulation.double_substation_top;
                    bottomSetpoint = Constants.Simulation.double_substation_bottom;
                    break;
                case 3:
                    topSetpoint = Constants.Simulation.score_floor_top;
                    bottomSetpoint = Constants.Simulation.score_floor_bottom;
                    break;
                case 4:
                    topSetpoint = Constants.Simulation.score_mid_top;
                    bottomSetpoint = Constants.Simulation.score_mid_bottom;
                    break;
                case 5:
                    topSetpoint = Constants.Simulation.score_high_top;
                    bottomSetpoint = Constants.Simulation.score_high_bottom;
                    break;
                default:
                    topSetpoint = Constants.Simulation.stowed_top;
                    bottomSetpoint = Constants.Simulation.stowed_bottom;
                    break;
              }
              
              // Here, we run PID control where the arm moves to the selected setpoint.
              pidOutputTop = topController.calculate(upEncoder.getDistance(), Units.degreesToRadians(topSetpoint - bottomSetpoint));
              upperNeoMotor.setVoltage(pidOutputTop);
              SmartDashboard.putNumber("Setpoint bottom (degrees)", bottomSetpoint);
              SmartDashboard.putNumber("Setpoint top (degrees)", topSetpoint);
              pidOutputBottom = bottomController.calculate(lowEncoder.getDistance(), Units.degreesToRadians(bottomSetpoint));
              lowerNeoMotor.setVoltage(pidOutputBottom);
              break;
          }
    }

    public void stopMotor(){
        // This just makes sure that our simulation code knows that the motor's off.
        upperNeoMotor.set(0.0);
    }

    // Actual Arm Code
    public Arm() {}

    public void setSpeedLower(double speed) {
        lowerNeoMotor.set(speed);
    }

    public void setSpeedUpper(double speed) {
        upperNeoMotor.set(speed);
    }

    public boolean raiseLevel() {
        if (level < 4) {
            level += 1;
            return false;
        } //else {  We're already returning, the code ends there, however i'm leaving this here incase this leads to an error
            return true;
        //}
    }

    public boolean lowerLevel() {
        if (level > 0) {
            level -= 1;
            return false;
        } //else { Same reasoning as above 
            return true;
        //}
    
    }
}
