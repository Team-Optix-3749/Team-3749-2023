package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSim;
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

  public ArmSimulationCommand(ArmSim armSim) {
    this.armSim = armSim;
    addRequirements(armSim);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    switch (armSim.controlMode.getSelected()) {
      case 1:
        // Here, we run PID control where the top arm acts like a four-bar relative to
        // the bottom.
        double pidOutputElbow = armSim.elbowController.calculate(armSim.elbowEncoder.getDistance(),
            Units.degreesToRadians(MathUtil.clamp(
                SmartDashboard.getNumber("Setpoint top (degrees)", 0)
                    - MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 150),
                        Constants.Arm.shoulder_min_angle, Constants.Arm.shoulder_max_angle),
                Constants.Arm.elbow_min_angle, Constants.Arm.elbow_max_angle)));
        armSim.leftElbowMotor.setVoltage(pidOutputElbow);

        double pidOutputShoulder = armSim.shoulderController.calculate(armSim.shoulderEncoder.getDistance(),
            Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0),
            Constants.Arm.shoulder_min_angle, Constants.Arm.shoulder_min_angle)));
        armSim.leftShoulderMotor.setVoltage(pidOutputShoulder);
        break;
      case 2:
        pidOutputElbow = armSim.elbowController.calculate(armSim.elbowEncoder.getDistance(),
            Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint top (degrees)", 0),
              Constants.Arm.shoulder_min_angle, Constants.Arm.shoulder_min_angle)));
        armSim.leftShoulderMotor.setVoltage(pidOutputElbow);

        pidOutputShoulder = armSim.shoulderController.calculate(armSim.shoulderEncoder.getDistance(),
            Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0),
            Constants.Arm.elbow_min_angle, Constants.Arm.elbow_min_angle)));
        armSim.leftShoulderMotor.setVoltage(pidOutputShoulder);
        break;
      default: // also case 0
        double topSetpoint, bottomSetpoint;
        switch (armSim.presetChooser.getSelected()) {
          case 0:
            topSetpoint = stowedTop;
            bottomSetpoint = stowedBottom;
            break;
          case 1:
            topSetpoint = intakeTop;
            bottomSetpoint = intakeBottom;
            break;
          case 2:
            topSetpoint = doubleSubstationTop;
            bottomSetpoint = doubleSubstationBottom;
            break;
          case 3:
            topSetpoint = scoreFloorTop;
            bottomSetpoint = scoreFloorBottom;
            break;
          case 4:
            topSetpoint = scoreMidTop;
            bottomSetpoint = scoreMidBottom;
            break;
          case 5:
            topSetpoint = scoreHighTop;
            bottomSetpoint = scoreHighBottom;
            break;
          default:
            topSetpoint = stowedTop;
            bottomSetpoint = stowedBottom;
            break;
        }
        // Here, we run PID control where the arm moves to the selected setpoint.
        pidOutputElbow = armSim.elbowController.calculate(armSim.elbowEncoder.getDistance(),
            Units.degreesToRadians(topSetpoint - bottomSetpoint));
        armSim.leftElbowMotor.setVoltage(pidOutputElbow);
        SmartDashboard.putNumber("Setpoint bottom (degrees)", bottomSetpoint);
        SmartDashboard.putNumber("Setpoint top (degrees)", topSetpoint);
        pidOutputShoulder = armSim.shoulderController.calculate(armSim.shoulderEncoder.getDistance(),
            Units.degreesToRadians(bottomSetpoint));
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
