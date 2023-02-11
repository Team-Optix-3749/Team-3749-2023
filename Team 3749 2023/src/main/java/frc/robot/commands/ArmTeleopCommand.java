package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utils.Xbox;

/**
 * Controls the elbow and shoulder using the left and right joysticks
 * 
 * @author Rohin Sood
 */
public class ArmTeleopCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Arm arm;

  private final Xbox operator;

  public ArmTeleopCommand(Arm armSim, Xbox operator) {
    this.arm = armSim;
    this.operator = operator;
    addRequirements(armSim);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double shoulderVoltage = 12 * operator.getRightY();
    double elbowVoltage = 12 * operator.getLeftY();

    arm.setShoulderVoltage(shoulderVoltage);
    arm.setElbowVoltage(elbowVoltage);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
