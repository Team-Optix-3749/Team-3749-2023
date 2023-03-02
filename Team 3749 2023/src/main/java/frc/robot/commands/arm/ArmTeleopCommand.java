package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmTrajectories;
import frc.robot.utils.Xbox;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

public class ArmTeleopCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final Xbox operator;
    private Trajectory trajectory;
    private State desiredState;
    private Timer timer = new Timer();
    private ArmSetpoints desiredSetpoint = ArmSetpoints.STOWED;
    private ArmSetpoints currentSetpoint = ArmSetpoints.STOWED;
    private boolean button_pressed = false;
    private boolean prev_button_pressed = false;

    public ArmTeleopCommand(Arm arm, Xbox operator) {
        this.arm = arm;
        this.operator = operator;
        trajectory = ArmTrajectories.getBlankTrajectory(arm.getArmCoordinate());
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();
        // System.out.println("START TRAJECTORY");
    }

    @Override
    public void execute() {
        button_pressed = false;
        if (operator.a().getAsBoolean()) {
            button_pressed = true;
            desiredSetpoint = ArmSetpoints.CONE_TOP;
        } else if (operator.b().getAsBoolean()) {
            button_pressed = true;
            desiredSetpoint = ArmSetpoints.CONE_MID;
        } else if (operator.rightBumper().getAsBoolean()) {
            button_pressed = true;
            desiredSetpoint = ArmSetpoints.STING;
        } else if (operator.rightTrigger().getAsBoolean()) {
            button_pressed = true;
            desiredSetpoint = ArmSetpoints.DOUBLE_SUBSTATION;
        }
        // if its a fresh button press, start a new trajectory
        if (prev_button_pressed == false && button_pressed == true) {
            trajectory = getTrajectory();
            timer.reset();
            timer.start();
        }
        if (trajectory.getTotalTimeSeconds() > timer.get()) {
            runTrajectory();
        }

        prev_button_pressed = button_pressed;

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {

        return false;
    }

    public void logging() {
        SmartDashboard.putNumber("CURRENT WAYPOINT X", desiredState.poseMeters.getTranslation().getX());
        SmartDashboard.putNumber("CURRENT WAYPOINT Y", desiredState.poseMeters.getTranslation().getY());

        SmartDashboard.putNumber("Arm Coordinate X", arm.getArmCoordinate().getX());
        SmartDashboard.putNumber("Arm Coordinate Y", arm.getArmCoordinate().getY());
    }

    private Trajectory getTrajectory() {
        Trajectory trajectory;

        if (currentSetpoint == ArmSetpoints.DOUBLE_SUBSTATION && desiredSetpoint != ArmSetpoints.STOWED) {
            System.out.println("Illigal double sub");
            return ArmTrajectories.getDoubleSubstationTrajectory(true);
        }
        if (currentSetpoint == ArmSetpoints.TOP_INTAKE && desiredSetpoint != ArmSetpoints.STOWED) {
            System.out.println("Illigal top intake");

            return ArmTrajectories.getGroundIntakeTrajectory(true);
        }

        if (desiredSetpoint == ArmSetpoints.CONE_TOP) {

            // if already there, reverse to stow
            if (desiredSetpoint == currentSetpoint) {
                System.out.println("reverse top node");

                currentSetpoint = ArmSetpoints.STOWED;
                trajectory = ArmTrajectories.getTopNodeTrajectory(true)
                        .concatenate(ArmTrajectories.getStingTrajectory(true));
                return trajectory;
            }
            // if at mid, run the node to node
            if (currentSetpoint == ArmSetpoints.CONE_MID) {
                System.out.println("mid to top node");

                trajectory = ArmTrajectories.getMidNodeToTopNodTrajectory(false);
            }
            // otherwise do it normally
            else {
                trajectory = ArmTrajectories.getTopNodeTrajectory(false);
                System.out.println("from sting to top node");

                // if at sting, run the trajectory. If not at sting, go to sting then run the
                // trajectory
                if (currentSetpoint != ArmSetpoints.STING) {

                    System.out.println("to sting to top node");
                    trajectory = ArmTrajectories.getStingTrajectory(false).concatenate(trajectory);
                }
            }
            // change the current setpoint and return
            currentSetpoint = ArmSetpoints.CONE_TOP;
            return trajectory;
        }

        else if (desiredSetpoint == ArmSetpoints.CONE_MID) {
            if (desiredSetpoint == currentSetpoint) {
                currentSetpoint = ArmSetpoints.STOWED;
                trajectory = ArmTrajectories.getMidNodeTrajectory(true)
                        .concatenate(ArmTrajectories.getStingTrajectory(true));
                return trajectory;
            }
            if (currentSetpoint == ArmSetpoints.CONE_TOP) {
                trajectory = ArmTrajectories.getMidNodeToTopNodTrajectory(true);
            } else {
                trajectory = ArmTrajectories.getMidNodeTrajectory(false);
                if (currentSetpoint != ArmSetpoints.STING) {
                    trajectory = ArmTrajectories.getStingTrajectory(false).concatenate(trajectory);
                }
            }
            currentSetpoint = ArmSetpoints.CONE_MID;
            return trajectory;
        }

        else if (desiredSetpoint == ArmSetpoints.DOUBLE_SUBSTATION) {
            if (desiredSetpoint == currentSetpoint) {
                currentSetpoint = ArmSetpoints.STOWED;

                trajectory = ArmTrajectories.getDoubleSubstationTrajectory(true);
                return trajectory;
            }
            trajectory = ArmTrajectories.getDoubleSubstationTrajectory(false);
            currentSetpoint = ArmSetpoints.DOUBLE_SUBSTATION;
            return trajectory;

        }

        else if (desiredSetpoint == ArmSetpoints.TOP_INTAKE) {
            if (desiredSetpoint == currentSetpoint) {
                currentSetpoint = ArmSetpoints.STOWED;

                trajectory = ArmTrajectories.getGroundIntakeTrajectory(true);
                return trajectory;
            }
            trajectory = ArmTrajectories.getGroundIntakeTrajectory(false);
            currentSetpoint = ArmSetpoints.TOP_INTAKE;
            return trajectory;
        }

        else if (desiredSetpoint == ArmSetpoints.STING) {
            if (desiredSetpoint == currentSetpoint) {
                currentSetpoint = ArmSetpoints.STOWED;

                trajectory = ArmTrajectories.getStingTrajectory(true);
                return trajectory;
            }
            trajectory = ArmTrajectories.getStingTrajectory(false);
            currentSetpoint = ArmSetpoints.STING;
            return trajectory;
        } else {
            return ArmTrajectories.getBlankTrajectory(arm.getArmCoordinate());
        }

    }

    private void runTrajectory() {
        double cur_time = timer.get();
        desiredState = trajectory.sample(cur_time);

        try {
            arm.setArmPosition(desiredState.poseMeters.getTranslation());
        } catch (Exception e) {
            System.out.println(e);
        }

        try {
            FileWriter myWriter = new FileWriter("data.csv", true);
            myWriter.write(String.valueOf(desiredState.poseMeters.getX()) + ','
                    + String.valueOf(desiredState.poseMeters.getY()) + '\n');
            myWriter.close();
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

        // System.out.println(
        // String.valueOf(desiredState.poseMeters.getX()) + ',' +
        // String.valueOf(desiredState.poseMeters.getY()) +" , " +
        // String.valueOf(timer.get()));

        logging();
    }
}
