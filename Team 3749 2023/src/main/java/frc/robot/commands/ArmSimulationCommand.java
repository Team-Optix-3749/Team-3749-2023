package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmSim;
import frc.robot.utils.Constants;
import frc.robot.utils.SmartData;

public class ArmSimulationCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final ArmSim armSim;

  private final SmartData<Double> elbowPIDOutput = new SmartData<Double>("elbowPIDOutput", 0.0);
  private final SmartData<Double> shoulderPIDOutput = new SmartData<Double>("shoulderPIDOutput", 0.0);

  private final ProfiledPIDController elbowController = new ProfiledPIDController(40.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(2, 5));
  private final ProfiledPIDController shoulderController = new ProfiledPIDController(40.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(2, 5));

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
        double pidOutputElbow = elbowController
            .calculate(
                armSim.getElbowEncoderDistance(), Units
                    .degreesToRadians(
                        MathUtil.clamp(
                            Constants.Arm.shoulderSetpoint.get()
                                - MathUtil.clamp(
                                    Constants.Arm.elbowSetpoint.get(),
                                    Constants.Arm.shoulder_min_angle,
                                    Constants.Arm.shoulder_max_angle),
                            Constants.Arm.elbow_min_angle, Constants.Arm.elbow_max_angle)));
        armSim.setElbowVoltage(pidOutputElbow);

        double pidOutputShoulder = shoulderController
            .calculate(
                armSim.getElbowEncoderDistance(), Units
                    .degreesToRadians(
                        MathUtil.clamp(
                            Constants.Arm.shoulderSetpoint.get()
                                - MathUtil.clamp(
                                    Constants.Arm.elbowSetpoint.get(),
                                    Constants.Arm.shoulder_min_angle,
                                    Constants.Arm.shoulder_max_angle),
                            Constants.Arm.elbow_min_angle, Constants.Arm.elbow_max_angle)));
        armSim.setShoulderVoltage(pidOutputShoulder);
        break;
      case 2:
        pidOutputElbow = elbowController
            .calculate(
                armSim.getElbowEncoderDistance(), Units
                    .degreesToRadians(
                        MathUtil.clamp(
                            Constants.Arm.elbowSetpoint.get(),
                            Constants.Arm.elbow_min_angle, Constants.Arm.elbow_max_angle)));
        armSim.setElbowVoltage(pidOutputElbow);

        pidOutputShoulder = shoulderController
            .calculate(
                armSim.getElbowEncoderDistance(), Units
                    .degreesToRadians(
                        MathUtil.clamp(
                            Constants.Arm.shoulderSetpoint.get(),
                            Constants.Arm.shoulder_min_angle, Constants.Arm.shoulder_max_angle)));
        armSim.setShoulderVoltage(pidOutputShoulder);
        break;
      default: // also case 0
        double elbowSetpoint, shoulderSetpoint;
        switch (armSim.presetChooser.getSelected()) {
          case 0:
            elbowSetpoint = stowedTop;
            shoulderSetpoint = stowedBottom;
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
            elbowSetpoint = stowedTop;
            shoulderSetpoint = stowedBottom;
            break;
        }
        SmartDashboard.putNumber("Elbow Setpoint", elbowSetpoint);
        SmartDashboard.putNumber("Shoulder Setpoint", shoulderSetpoint);

        // Here, we run PID control where the arm moves to the selected setpoint.
        pidOutputElbow = elbowController.calculate(armSim.getElbowEncoderDistance(),
            Units.degreesToRadians(elbowSetpoint - shoulderSetpoint));
        elbowPIDOutput.set(pidOutputElbow);

        armSim.setElbowVoltage(pidOutputElbow);

        pidOutputShoulder = shoulderController.calculate(armSim.getShoulderEncoderDistance(),
            Units.degreesToRadians(shoulderSetpoint));
        shoulderPIDOutput.set(pidOutputShoulder);

        armSim.setShoulderVoltage(pidOutputShoulder);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSim.setElbowVoltage(0.0);
    armSim.setShoulderVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
