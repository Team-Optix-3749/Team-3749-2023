package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmTrajectories.ArmPaths;
import frc.robot.subsystems.intake.ArmIntake;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;;

/***
 * @author Noah Simon
 * 
 * Moves the arm! It goes between setpoints listed in Constants.Arm.ArmSetpoints and 
 * travels with ArmPaths listed in ArmTrajectories. Uses Trajectory objects created 
 * in ArmTrajectories for motion profiling. Additionally controls the intake during 
 * the paths as described in the ArmPath
 */

public class MoveArm extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final ArmIntake intake;
    private final ArmSetpoints desiredSetpoint;
    private State desiredState;
    private Timer timer = new Timer();
    private ArmPaths trajectoryInformation;
    private int trajectoryIndex = 0;

    public MoveArm(Arm arm, ArmIntake intake, ArmSetpoints setpoint) {
        this.arm = arm;
        this.intake = intake;
        this.desiredSetpoint = setpoint;
        setName(setpoint.toString() + " Trajectory");
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        trajectoryInformation = findTrajectory(desiredSetpoint, arm);
        trajectoryIndex = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() < trajectoryInformation.pauseLengths[trajectoryIndex]) {
            intake.setVoltage(Constants.ArmIntake.idleVoltage);
            return;
        }
        intake.setVoltage(trajectoryInformation.intakeVoltages[trajectoryIndex]);

        double cur_time = timer.get();
        desiredState = trajectoryInformation.trajectories[trajectoryIndex]
                .sample(cur_time - trajectoryInformation.pauseLengths[trajectoryIndex]);

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
            // System.out.println("An error occurred.");
            // e.printStackTrace();
        }
        System.out.println(
                String.valueOf(desiredState.poseMeters.getX()) + ',' + String.valueOf(desiredState.poseMeters.getY()));
        logging();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(Constants.ArmIntake.idleVoltage);
    }

    @Override
    public boolean isFinished() {
        // if the current trajectory is over
        if (trajectoryInformation.trajectoryLengths[trajectoryIndex]
                + trajectoryInformation.pauseLengths[trajectoryIndex] < timer.get()) {
            // if we are not at the last trajectory in the list, start the next one
            if (trajectoryIndex < trajectoryInformation.numTrajectories - 1) {
                trajectoryIndex++;
                timer.reset();
                timer.start();
            } else {
                return true;
            }
        }
        return false;

    }

    public void logging() {
        Constants.Arm.currWaypointX.set(desiredState.poseMeters.getTranslation().getX());
        Constants.Arm.currWaypointY.set(desiredState.poseMeters.getTranslation().getY());
        
        Constants.Arm.armCoordinateX.set(arm.getArmCoordinate().getX());
        Constants.Arm.armCoordinateY.set(arm.getArmCoordinate().getY());
    }

    /***
     * 
     * @param desiredSetpoint ArmSetpoints: where you want to go
     * @param arm             Arm: the arm subsystem object, used for its position
     *                        and getting/setting current setpoint
     * @return Trajectory: the optimal trajectory for where you are to where you
     *         want to go
     */
    private ArmPaths findTrajectory(ArmSetpoints desiredSetpoint, Arm arm) {
        ArmSetpoints currentSetpoint = arm.getCurrentSetpoint();
        // you should only be able to go to stowed from double substation and ground
        // intake
        if (currentSetpoint == ArmSetpoints.DOUBLE_SUBSTATION && desiredSetpoint != ArmSetpoints.STOW) {
            arm.setCurrentSetpoint(ArmSetpoints.STOW);
            return ArmPaths.DOUBLESUB_TO_STOW;
        }
        if (currentSetpoint == ArmSetpoints.GROUND_INTAKE && desiredSetpoint != ArmSetpoints.STOW) {
            arm.setCurrentSetpoint(ArmSetpoints.STOW);
            return ArmPaths.GROUND_INTAKE_TO_STOW;
        }

        switch (desiredSetpoint) {
            case PLACE_TOP:
                // if already there, place and return
                if (desiredSetpoint == currentSetpoint) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return ArmPaths.TOP_TO_STOW;
                }
                // if at mid, run the node to node
                else if (currentSetpoint == ArmSetpoints.PLACE_MID) {
                    arm.setCurrentSetpoint(ArmSetpoints.PLACE_TOP);
                    return ArmPaths.MID_TO_TOP;
                }
                // otherwise do it normally
                else {
                    arm.setCurrentSetpoint(ArmSetpoints.PLACE_TOP);
                    // if at sting, run the trajectory. If not at sting, go to sting then run the
                    // trajectory
                    if (currentSetpoint == ArmSetpoints.STOW) {
                        return ArmPaths.STOW_TO_TOP;
                    }
                    return ArmPaths.STING_TO_TOP;
                }

            case PLACE_MID:
                if (desiredSetpoint == currentSetpoint) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return ArmPaths.MID_TO_STOW;
                }
                if (currentSetpoint == ArmSetpoints.PLACE_TOP) {
                    arm.setCurrentSetpoint(ArmSetpoints.PLACE_MID);
                    return ArmPaths.TOP_TO_MID;
                } else {
                    arm.setCurrentSetpoint(ArmSetpoints.PLACE_MID);
                    if (currentSetpoint != ArmSetpoints.STING) {
                        return ArmPaths.STOW_TO_MID;
                    }
                    return ArmPaths.STING_TO_MID;
                }

            case DOUBLE_SUBSTATION:
                if (desiredSetpoint == currentSetpoint) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return ArmPaths.DOUBLESUB_TO_STOW;
                } else {
                    arm.setCurrentSetpoint(ArmSetpoints.DOUBLE_SUBSTATION);
                    return ArmPaths.STOW_TO_DOUBLESUB;
                }

            case GROUND_INTAKE:
                if (desiredSetpoint == currentSetpoint) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return ArmPaths.GROUND_INTAKE_TO_STOW;
                } else {
                    arm.setCurrentSetpoint(ArmSetpoints.GROUND_INTAKE);
                    return ArmPaths.STOW_TO_GROUND_INTAKE;
                }

            case STING:
                arm.setCurrentSetpoint(ArmSetpoints.STING);
                if (desiredSetpoint == currentSetpoint) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return ArmPaths.STING_TO_STOW;
                } else if (currentSetpoint == ArmSetpoints.PLACE_TOP)
                    return ArmPaths.TOP_TO_STING;
                else if (currentSetpoint == ArmSetpoints.PLACE_MID)
                    return ArmPaths.MID_TO_STING;
                else
                    return ArmPaths.STOW_TO_STING;

            default:
                return null;
        }
    }
}
