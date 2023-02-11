package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.*;
import frc.robot.utils.Constants.Arm.*;
import frc.robot.utils.Constants;

/**
 * Sets the joint setpoints for the double jointed arm sim using a preset chooser & profiled PID controller
 * 
 * @author Rohin Sood
*/
public class ArmSimCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Arm armSim;

  private static final double stowedBottom = 90;
  private static final double stowedTop = 260;

  private static final double intakeBottom = 135;
  private static final double intakeTop = 265;

  private static final double doubleSubstationBottom = 60;
  private static final double doubleSubstationTop = 185;

  private static final double scoreFloorBottom = 120;
  private static final double scoreFloorTop = 255;

  private static final double scoreMidBottom = 95;
  private static final double scoreMidTop = 195;

  private static final double scoreHighBottom = 135;
  private static final double scoreHighTop = 160;

  private ProfiledPIDController elbowController = new ProfiledPIDController(Constants.Arm.elbowKP.get(), Constants.Arm.elbowKI.get(), Constants.Arm.elbowKD.get(),
    new TrapezoidProfile.Constraints(2, 5));
  private ProfiledPIDController shoulderController = new ProfiledPIDController(Constants.Arm.shoulderKP.get(), Constants.Arm.shoulderKI.get(), Constants.Arm.shoulderKD.get(),
    new TrapezoidProfile.Constraints(2, 5));

  private SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
	private SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

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
		presetChooser.addOption("Floor Intake Position", 1);
		presetChooser.addOption("Double Substation Intake", 2);
		presetChooser.addOption("Floor Node Score", 3);
		presetChooser.addOption("Mid Node Score", 4);
		presetChooser.addOption("High Node Score", 5);
		SmartDashboard.putData(controlMode);
		SmartDashboard.putData(presetChooser);

  }

  @Override
  public void execute() {

    double elbowSetpoint, shoulderSetpoint;
    switch (presetChooser.getSelected()) {
      case 0:
        elbowSetpoint = ElbowSetpoints.STOWED.angle;
        shoulderSetpoint = ShoulderSetpoints.STOWED.angle;
        break;
      case 1:
        elbowSetpoint = intakeTop;
        shoulderSetpoint = intakeBottom;
        break;
      case 2:
        elbowSetpoint = doubleSubstationTop;
        shoulderSetpoint = doubleSubstationBottom;
        break;
      case 3:
        elbowSetpoint = scoreFloorTop;
        shoulderSetpoint = scoreFloorBottom;
        break;
      case 4:
        elbowSetpoint = scoreMidTop;
        shoulderSetpoint = scoreMidBottom;
        break;
      case 5:
        elbowSetpoint = scoreHighTop;
        shoulderSetpoint = scoreHighBottom;
        break;
      default:
      elbowSetpoint = ElbowSetpoints.STOWED.angle;
      shoulderSetpoint = ShoulderSetpoints.STOWED.angle;
      break;
    }
    // Here, we run PID control where the arm moves to the selected setpoint.
    double pidOutputElbow = elbowController.calculate(armSim.getElbowPosition(),
        Units.degreesToRadians(elbowSetpoint - shoulderSetpoint));
    armSim.setElbowVoltage(pidOutputElbow);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", shoulderSetpoint);
    SmartDashboard.putNumber("Setpoint top (degrees)", elbowSetpoint);
    double pidOutputShoulder = shoulderController.calculate(armSim.getShoulderPosition(),
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
