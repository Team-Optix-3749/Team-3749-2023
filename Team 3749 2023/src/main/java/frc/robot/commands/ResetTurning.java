package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetTurning extends CommandBase {
  
  SwerveSubsystem swerveSubsystem;

  public ResetTurning(SwerveSubsystem swerveSubsystem){
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void execute() {
    swerveSubsystem.resetEncoders();

  }

}
