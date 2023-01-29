package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSim;
import frc.robot.utils.Constants;
import frc.robot.utils.SmartData;

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
    armSim.setElbow(0.0);
    armSim.setShoulder(0.0);
  }

  @Override
  public void execute() {
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
        Constants.Arm.elbowSetpoint.set(this.elbowSetpoint);
        Constants.Arm.shoulderSetpoint.set(this.shoulderSetpoint);

        armSim.setElbow(Units.degreesToRadians(elbowSetpoint));

        armSim.setShoulder(Units.degreesToRadians(shoulderSetpoint));
  }

  @Override
  public void end(boolean interrupted) {
    armSim.setElbow(0.0);
    armSim.setShoulder(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
