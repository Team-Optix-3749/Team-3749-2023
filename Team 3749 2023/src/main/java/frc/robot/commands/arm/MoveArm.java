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
import frc.robot.subsystems.claw.Claw;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;;

public class MoveArm extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final Claw claw;
    private final ArmSetpoints desiredSetpoint;
    private Trajectory[] trajectories;
    private State desiredState;
    private Timer timer = new Timer();
    private double trajectoryLengths[];
    private int trajectoryIndex = 0;
    private int numTrajectories = 0;
    private double pauseLengths[];
    private double[] clawVoltages;

    public MoveArm(Arm arm, Claw claw, ArmSetpoints setpoint) {
        this.arm = arm;
        this.claw = claw;
        this.desiredSetpoint = setpoint;
        // this.trajectory = trajectory;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        trajectories = findTrajectory(desiredSetpoint, arm);
        numTrajectories = trajectories.length;
        trajectoryIndex = 0;
        timer.reset();
        timer.start();
        // System.out.println("START TRAJECTORY");
    }

    @Override
    public void execute() {
        if (timer.get() < pauseLengths[trajectoryIndex]) {
            claw.setVoltage(Constants.Claw.idleVoltage);
            return;
        }
        claw.setVoltage(clawVoltages[trajectoryIndex]);

        double cur_time = timer.get();
        desiredState = trajectories[trajectoryIndex].sample(cur_time - pauseLengths[trajectoryIndex]);

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
        // System.out.println(clawVoltages[trajectoryIndex]);
        System.out.println(
                String.valueOf(desiredState.poseMeters.getX()) + ',' + String.valueOf(desiredState.poseMeters.getY()));
        logging();
    }

    @Override
    public void end(boolean interrupted) {
        claw.setVoltage(Constants.Claw.idleVoltage);
    }

    @Override
    public boolean isFinished() {
        // if the current trajectory is over
        if (trajectoryLengths[trajectoryIndex] + pauseLengths[trajectoryIndex] < timer.get()) {
            // if we are not at the last trajectory in the list, start the next one
            if (trajectoryIndex < numTrajectories - 1) {
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
        SmartDashboard.putNumber("CURRENT WAYPOINT X", desiredState.poseMeters.getTranslation().getX());
        SmartDashboard.putNumber("CURRENT WAYPOINT Y", desiredState.poseMeters.getTranslation().getY());

        SmartDashboard.putNumber("Arm Coordinate X", arm.getArmCoordinate().getX());
        SmartDashboard.putNumber("Arm Coordinate Y", arm.getArmCoordinate().getY());
    }

    /***
     * 
     * @param desiredSetpoint ArmSetpoints: where you want to go
     * @param arm             Arm: the arm subsystem object, used for its position
     *                        and getting/setting current setpoint
     * @return Trajectory: the optimal trajectory for where you are to where you
     *         want to go
     */
    private Trajectory[] findTrajectory(ArmSetpoints desiredSetpoint, Arm arm) {
        ArmSetpoints currentSetpoint = arm.getCurrentSetpoint();
        // you should only be able to go to stowed from double substation and ground
        // intake
        if (currentSetpoint == ArmSetpoints.DOUBLE_SUBSTATION && desiredSetpoint != ArmSetpoints.STOWED) {
            System.out.println("Illigal double sub");
            arm.setCurrentSetpoint(ArmSetpoints.STOWED);
            Trajectory trajectory = ArmTrajectories.getDoubleSubstationTrajectory(true);
            trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
            clawVoltages = new double[] { Constants.Claw.idleVoltage };
            pauseLengths = new double[] { 0 };
            return new Trajectory[] { trajectory };
        }
        if (currentSetpoint == ArmSetpoints.TOP_INTAKE && desiredSetpoint != ArmSetpoints.STOWED) {
            System.out.println("Illigal top intake");
            arm.setCurrentSetpoint(ArmSetpoints.STOWED);
            Trajectory trajectory = ArmTrajectories.getGroundIntakeTrajectory(true);
            trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
            clawVoltages = new double[] { Constants.Claw.idleVoltage };
            pauseLengths = new double[] { 0 };
            return new Trajectory[] { trajectory };
        }

        if (desiredSetpoint == ArmSetpoints.CONE_TOP) {

            // if already there, place and return
            if (desiredSetpoint == currentSetpoint) {
                System.out.println("reverse top node");
 
                arm.setCurrentSetpoint(ArmSetpoints.STOWED);
                Trajectory[] trajectories = new Trajectory[] {
                        ArmTrajectories.getTopNodePlaceDownTrajectory(false),
                        ArmTrajectories.getTopNodePlaceReturnTrajectory(false).concatenate(
                                ArmTrajectories.getStingTrajectory(true))
                };

                trajectoryLengths = new double[] {
                        trajectories[0].getTotalTimeSeconds(),
                        trajectories[1].getTotalTimeSeconds(),

                };
                clawVoltages = new double[] { Constants.Claw.idleVoltage, Constants.Claw.releaseObjectVoltage};
                pauseLengths = new double[] { 0, 0.5 };
                return trajectories;

            }
            // if at mid, run the node to node
            else if (currentSetpoint == ArmSetpoints.CONE_MID) {
                System.out.println("mid to top node");
                arm.setCurrentSetpoint(ArmSetpoints.CONE_TOP);
                Trajectory trajectory = ArmTrajectories.getMidNodeToTopNodeTrajectory(false);
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };
            }
            // otherwise do it normally
            else {
                arm.setCurrentSetpoint(ArmSetpoints.CONE_TOP);
                Trajectory trajectory = ArmTrajectories.getTopNodeTrajectory(false);
                System.out.println("from sting to top node");

                // if at sting, run the trajectory. If not at sting, go to sting then run the
                // trajectory
                if (currentSetpoint != ArmSetpoints.STING) {
                    System.out.println("to sting to top node");
                    trajectory = ArmTrajectories.getStingTrajectory(false).concatenate(trajectory);
                }
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };
            }
            // change the current setpoint and return
        }

        else if (desiredSetpoint == ArmSetpoints.CONE_MID) {
            if (desiredSetpoint == currentSetpoint) {
                arm.setCurrentSetpoint(ArmSetpoints.STOWED);
                Trajectory[] trajectories = new Trajectory[] {
                        ArmTrajectories.getMidNodePlaceDownTrajectory(false),
                        ArmTrajectories.getMidNodePlaceReturnTrajectory(false).concatenate(
                                ArmTrajectories.getStingTrajectory(true))
                };
                trajectoryLengths = new double[] {
                        trajectories[0].getTotalTimeSeconds(),
                        trajectories[1].getTotalTimeSeconds(),
                };
                clawVoltages = new double[] { Constants.Claw.idleVoltage, Constants.Claw.releaseObjectVoltage };
                pauseLengths = new double[] { 0, 0.5 };
                return trajectories;
            }

            if (currentSetpoint == ArmSetpoints.CONE_TOP) {
                arm.setCurrentSetpoint(ArmSetpoints.CONE_MID);
                Trajectory trajectory = ArmTrajectories.getMidNodeToTopNodeTrajectory(true);
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };
            } else {
                arm.setCurrentSetpoint(ArmSetpoints.CONE_MID);
                Trajectory trajectory;
                trajectory = ArmTrajectories.getMidNodeTrajectory(false);
                if (currentSetpoint != ArmSetpoints.STING) {
                    trajectory = ArmTrajectories.getStingTrajectory(false).concatenate(trajectory);
                }
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };
            }
        }

        else if (desiredSetpoint == ArmSetpoints.DOUBLE_SUBSTATION) {
            if (desiredSetpoint == currentSetpoint) {
                Trajectory trajectory = ArmTrajectories.getDoubleSubstationTrajectory(true);
                arm.setCurrentSetpoint(ArmSetpoints.STOWED);
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };
            } else {
                Trajectory trajectory = ArmTrajectories.getDoubleSubstationTrajectory(false);
                arm.setCurrentSetpoint(ArmSetpoints.DOUBLE_SUBSTATION);
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };

            }

        }

        else if (desiredSetpoint == ArmSetpoints.TOP_INTAKE) {
            if (desiredSetpoint == currentSetpoint) {
                Trajectory trajectory = ArmTrajectories.getGroundIntakeTrajectory(true);
                arm.setCurrentSetpoint(ArmSetpoints.STOWED);
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.intakeVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };
            } else {
                Trajectory trajectory = ArmTrajectories.getGroundIntakeTrajectory(false);
                arm.setCurrentSetpoint(ArmSetpoints.TOP_INTAKE);
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };
            }
        }

        else if (desiredSetpoint == ArmSetpoints.STING) {
            if (desiredSetpoint == currentSetpoint) {
                Trajectory trajectory = ArmTrajectories.getStingTrajectory(true);
                arm.setCurrentSetpoint(ArmSetpoints.STOWED);
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };
            } 
            else if (currentSetpoint == ArmSetpoints.CONE_TOP){
                arm.setCurrentSetpoint(ArmSetpoints.STING);
                Trajectory trajectory = ArmTrajectories.getTopNodeTrajectory(true);
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };
            }
            else if (currentSetpoint == ArmSetpoints.CONE_MID){
                arm.setCurrentSetpoint(ArmSetpoints.STING);
                Trajectory trajectory = ArmTrajectories.getMidNodeTrajectory(true);
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                return new Trajectory[] { trajectory };
            }
            else {
                
                Trajectory trajectory = ArmTrajectories.getStingTrajectory(false);
                trajectoryLengths = new double[] { trajectory.getTotalTimeSeconds() };
                clawVoltages = new double[] { Constants.Claw.idleVoltage };
                pauseLengths = new double[] { 0 };
                arm.setCurrentSetpoint(ArmSetpoints.STING);
                return new Trajectory[] { trajectory };

            }
        } else {
            return null;
        }
    }
}
