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

public class ArmTeleopCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Arm arm;

  private final Xbox operator;

  private ProfiledPIDController elbowController = new ProfiledPIDController(Constants.Arm.elbowKP.get(),
  Constants.Arm.elbowKI.get(), Constants.Arm.elbowKD.get(),
  new TrapezoidProfile.Constraints(2, 5));
  private ProfiledPIDController shoulderController = new ProfiledPIDController(Constants.Arm.shoulderKP.get(),
  Constants.Arm.shoulderKI.get(), Constants.Arm.shoulderKD.get(),
      new TrapezoidProfile.Constraints(2, 5));

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
    
    // switch (Constants.Arm.controlMode.getSelected()) {
      //   case 1:
      
      double shoulderVoltage = 12 * operator.getRightY();
      double elbowVoltage = 12 * operator.getLeftY();
      
      arm.setShoulderVoltage(shoulderVoltage);
      arm.setElbowVoltage(elbowVoltage);

  //       break;
  //     default: // also case 0

  //       double elbowSetpoint, shoulderSetpoint;

  //       if (operator.getLeftTrigger()) { // cube setpoints
  //         if (operator.getRawButton(Button.kX.value)) { // mid
  //           elbowSetpoint = 0;
  //           shoulderSetpoint = 0;
  //         }
  //         if (operator.getRawButton(Button.kY.value)) { // high
  //           elbowSetpoint = 0;
  //           shoulderSetpoint = 0;
  //         }
  //       } else if (operator.getRightTrigger()) { // cone setpoints
  //         if (operator.getRawButton(Button.kX.value)) { // mid
  //           elbowSetpoint = 0;
  //           shoulderSetpoint = 0;
  //         }
  //         if (operator.getRawButton(Button.kY.value)) { // high
  //           elbowSetpoint = 0;
  //           shoulderSetpoint = 0;
  //         }
  //       } else if (operator.getRawButton(Button.kA.value)) { // floor
  //         elbowSetpoint = 0;
  //         shoulderSetpoint = 0;
  //       } else if (operator.getRawButton(Button.kB.value)) { // stowing
  //         elbowSetpoint = ElbowSetpoints.STOWED.angle;
  //         shoulderSetpoint = ShoulderSetpoints.STOWED.angle;
  //       } else {
  //         switch (ArmSim.presetChooser.getSelected()) {
  //           case 0:
  //             elbowSetpoint = ElbowSetpoints.STOWED.angle;
  //             shoulderSetpoint = ShoulderSetpoints.STOWED.angle;
  //             break;
  //           default:
  //             elbowSetpoint = ElbowSetpoints.STOWED.angle;
  //             shoulderSetpoint = ShoulderSetpoints.STOWED.angle;
  //             break;
  //         }

  //         arm.setShoulderVoltage(pidOutputShoulder);
  //       }
  //       break;
  //   }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
