package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.*;
import frc.robot.utils.Constants.Arm.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Kinematics;
import frc.robot.utils.SmartData;

/**
 * Sets the joint setpoints for the double jointed arm sim using a preset
 * chooser & profiled PID controller
 * 
 * @author Rohin Sood
 */
public class ArmSimCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Arm armSim;

  private ProfiledPIDController elbowController = new ProfiledPIDController(Constants.Arm.elbowKP.get(),
      Constants.Arm.elbowKI.get(), Constants.Arm.elbowKD.get(),
      new TrapezoidProfile.Constraints(2, 5));
  private ProfiledPIDController shoulderController = new ProfiledPIDController(Constants.Arm.shoulderKP.get(),
      Constants.Arm.shoulderKI.get(), Constants.Arm.shoulderKD.get(),
      new TrapezoidProfile.Constraints(2, 5));

  private SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
  private SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

  // desired positions for x and y
  public static SmartData<Double> xf = new SmartData<Double>("desired x position", 55.0);
  public static SmartData<Double> yf = new SmartData<Double>("desired y position", 0.0);

  public ArmSimCommand(Arm armSim) {
    this.armSim = armSim;
    addRequirements(armSim);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Setpoint top (degrees)", 90);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", 90);
    controlMode.setDefaultOption("Presets (Setpoints)", 0);
    controlMode.addOption("Virtual Four Bar", 1);
    controlMode.addOption("Manual Angle Adjust", 2);

    presetChooser.setDefaultOption("Starting Position", 0);
    presetChooser.addOption("Ground Intake Position", 1);
    presetChooser.addOption("Inverse Kinematics Position", 2);
    SmartDashboard.putData(controlMode);
    SmartDashboard.putData(presetChooser);

  }

  @Override
  public void execute(){

    double elbowSetpoint, shoulderSetpoint;
    switch (presetChooser.getSelected()) {
      case 0:
        elbowSetpoint = ElbowSetpoints.ZERO.angle + 180;
        shoulderSetpoint = ShoulderSetpoints.ZERO.angle + 180;
        break;
      case 1:
        elbowSetpoint = ElbowSetpoints.GROUND_INTAKE.angle;
        shoulderSetpoint = ShoulderSetpoints.GROUND_INTAKE.angle;
        break;
      case 2:
        Kinematics kinematics = new Kinematics();
        Pair<Double, Double> angles = kinematics.inverse(xf.get(), yf.get());

        elbowSetpoint = 180-Math.toDegrees(angles.getSecond());
        shoulderSetpoint = 180-Math.toDegrees(angles.getFirst());
        break;
      default:
        elbowSetpoint = ElbowSetpoints.ZERO.angle;
        shoulderSetpoint = ShoulderSetpoints.ZERO.angle;
        break;
    }

    double pidOutputElbow = elbowController.calculate(armSim.getElbowDistance(),
        Units.degreesToRadians(elbowSetpoint - shoulderSetpoint));
    armSim.setElbowVoltage(pidOutputElbow);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", shoulderSetpoint);
    SmartDashboard.putNumber("Setpoint top (degrees)", elbowSetpoint);
    double pidOutputShoulder = shoulderController.calculate(armSim.getShoulderDistance(),
        Units.degreesToRadians(shoulderSetpoint));
    armSim.setShoulderVoltage(pidOutputShoulder);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
