package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSim;
import frc.robot.utils.Constants.Arm.*;
import frc.robot.utils.Constants;

public class ArmSimulationCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final ArmSim armSim;

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

  private double elbowSetpoint, shoulderSetpoint;

  public ProfiledPIDController elbowController = new ProfiledPIDController(Constants.Arm.elbowKP.get(), Constants.Arm.elbowKI.get(), Constants.Arm.elbowKD.get(),
  new TrapezoidProfile.Constraints(2, 5));
public ProfiledPIDController shoulderController = new ProfiledPIDController(Constants.Arm.shoulderKP.get(), Constants.Arm.shoulderKI.get(), Constants.Arm.shoulderKD.get(),
  new TrapezoidProfile.Constraints(2, 5));

  public ArmSimulationCommand(ArmSim armSim) {
    this.armSim = armSim;
    addRequirements(armSim);
  }

  @Override
  public void initialize() {
    elbowController =  new ProfiledPIDController(Constants.Arm.elbowKP.get(), Constants.Arm.elbowKI.get(), Constants.Arm.elbowKD.get(),
    new TrapezoidProfile.Constraints(2, 5));
    shoulderController = new ProfiledPIDController(Constants.Arm.shoulderKP.get(), Constants.Arm.shoulderKI.get(), Constants.Arm.shoulderKD.get(),
  new TrapezoidProfile.Constraints(2, 5));

  }

  @Override
  public void execute() {

    switch (armSim.controlMode.getSelected()) {
      case 1:
        // Here, we run PID control where the top arm acts like a four-bar relative to
        // the bottom.
        double pidOutputElbow = elbowController.calculate(armSim.elbowEncoder.getDistance(),
            Units.degreesToRadians(MathUtil.clamp(
                SmartDashboard.getNumber("Setpoint top (degrees)", 0)
                    - MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 150),
                        Constants.Arm.shoulder_min_angle, Constants.Arm.shoulder_max_angle),
                Constants.Arm.elbow_min_angle, Constants.Arm.elbow_max_angle)));
        armSim.leftElbowMotor.setVoltage(pidOutputElbow);

        double pidOutputShoulder = shoulderController.calculate(armSim.shoulderEncoder.getDistance(),
            Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0),
                Constants.Arm.shoulder_min_angle, Constants.Arm.shoulder_min_angle)));
        armSim.leftShoulderMotor.setVoltage(pidOutputShoulder);
        break;
      case 2:
        pidOutputElbow = elbowController.calculate(armSim.elbowEncoder.getDistance(),
            Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint top (degrees)", 0),
                Constants.Arm.shoulder_min_angle, Constants.Arm.shoulder_min_angle)));
        armSim.leftShoulderMotor.setVoltage(pidOutputElbow);

        pidOutputShoulder = shoulderController.calculate(armSim.shoulderEncoder.getDistance(),
            Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0),
                Constants.Arm.elbow_min_angle, Constants.Arm.elbow_min_angle)));
        armSim.leftShoulderMotor.setVoltage(pidOutputShoulder);
        break;
      default: // also case 0
        double elbowSetpoint, shoulderSetpoint;
        switch (armSim.presetChooser.getSelected()) {
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
        pidOutputElbow = elbowController.calculate(armSim.elbowEncoder.getDistance(),
            Units.degreesToRadians(elbowSetpoint - shoulderSetpoint));
        armSim.leftElbowMotor.setVoltage(pidOutputElbow);
        SmartDashboard.putNumber("Setpoint bottom (degrees)", shoulderSetpoint);
        SmartDashboard.putNumber("Setpoint top (degrees)", elbowSetpoint);
        pidOutputShoulder = shoulderController.calculate(armSim.shoulderEncoder.getDistance(),
            Units.degreesToRadians(shoulderSetpoint));
        armSim.leftShoulderMotor.setVoltage(pidOutputShoulder);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
