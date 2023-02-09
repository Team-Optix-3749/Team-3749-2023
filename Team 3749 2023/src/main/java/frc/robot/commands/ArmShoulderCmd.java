package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.Arm.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Xbox;
// import frc.robot.utils.ArmSim;

public class ArmShoulderCmd extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Arm arm;

  public ArmShoulderCmd(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("desired shoulder voltage", 0);
    arm.setShoulderVoltage(0);
  }

  @Override
  public void execute() {
    arm.setShoulderVoltage(SmartDashboard.getNumber("desired shoulder voltage", 0));
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("desired shoulder voltage", 0);
    arm.setShoulderVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
