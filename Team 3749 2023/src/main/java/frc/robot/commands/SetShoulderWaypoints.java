package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utils.Xbox;

/**
 * Controls the elbow and shoulder using the left and right joysticks
 * 
 * @author Rohin Sood
 */
public class SetShoulderWaypoints extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Arm arm;

  private double first_waypoint;
  private double second_waypoint;
  private double waypoint;

  private boolean finished = false;
  private double waypoint_reached = 0;

  public SetShoulderWaypoints(Arm arm, double first_waypoint, double second_waypoint) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {

    arm.setShoulderAngle(waypoint);

    if(waypoint_reached == 2) {
      finished = true;
    }

    if (arm.getShoulderAtSetpoint(waypoint)) {
      waypoint_reached++;
      waypoint = second_waypoint;
    } else {
      waypoint = first_waypoint;
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
