package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmTrajectories;
import frc.robot.subsystems.intake.ArmIntake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.LEDs.LEDPattern;;

/***
 * @author Noah Simon
 * 
 *         Moves the arm! It goes between setpoints listed in
 *         Constants.Arm.ArmSetpoints and
 *         travels with ArmPaths listed in ArmTrajectories. Uses Trajectory
 *         objects created
 *         in ArmTrajectories for motion profiling. Additionally controls the
 *         intake during
 *         the paths as described in the ArmPath
 */

public class MoveArm extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final LEDs leds;
    private final ArmSetpoints desiredSetpoint;
    private State desiredState;
    private Timer timer = new Timer();
    private int trajectoryIndex = 0;
    private boolean fromAlign = false;
    private ArmTrajectories armTrajectories;
    private Trajectory[] trajectories;

    public MoveArm(ArmSetpoints setpoint) {
        this.arm = Robot.arm;
        this.armTrajectories = armTrajectories;
        this.desiredSetpoint = setpoint;
        this.leds = Robot.leds;
        setName(setpoint.toString() + " Trajectory");
        addRequirements(arm);
    }

    public MoveArm(ArmSetpoints setpoint,
            boolean fromAlign) {
        this.arm = Robot.arm;
        this.armTrajectories = Arm.armTrajectories;

        this.desiredSetpoint = setpoint;
        this.leds = Robot.leds;
        this.fromAlign = fromAlign;
        setName(setpoint.toString() + " Trajectory");
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        System.out.println(desiredSetpoint.name());
        trajectories = findTrajectory(desiredSetpoint, arm);
        trajectoryIndex = 0;
        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {
        if (trajectories == null) {
            System.out.println("NO SETPOINT");
            return;
        }

        double cur_time = timer.get();
        desiredState = trajectories[trajectoryIndex]
                .sample(cur_time);
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
        logging();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (trajectories == null) {
            System.out.println("NO SETPOINT");
            return true;
        }
        // if the current trajectory is over
        if (trajectories[trajectoryIndex].getTotalTimeSeconds() < timer.get()) {
            // if we are not at the last trajectory in the list, start the next one
            if (trajectoryIndex < trajectories.length - 1) {
                trajectoryIndex++;
                timer.reset();
                timer.start();
            } else {
                System.out.println("DDDDDOOOONNNNNNEEEEEE");
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
    private Trajectory[] findTrajectory(ArmSetpoints desiredSetpoint, Arm arm) {
        ArmSetpoints currentSetpoint = arm.getCurrentSetpoint();
        // you should only be able to go to stowed from double substation and ground
        // intake
        if (currentSetpoint == ArmSetpoints.DOUBLE_SUBSTATION_CONE
                && (desiredSetpoint != ArmSetpoints.STOW || desiredSetpoint != ArmSetpoints.CUBE_STOW)) {
            arm.setCurrentSetpoint(ArmSetpoints.STOW);
            leds.setLEDPattern(leds.getDefaultColor());
            return new Trajectory[] { armTrajectories.getDoubleSubConeToStow() };
        }

        if (currentSetpoint == ArmSetpoints.DOUBLE_SUBSTATION_CUBE
                && (desiredSetpoint != ArmSetpoints.STOW || desiredSetpoint != ArmSetpoints.CUBE_STOW)) {
            arm.setCurrentSetpoint(ArmSetpoints.STOW);
            leds.setLEDPattern(leds.getDefaultColor());
            return new Trajectory[] { armTrajectories.getDoubleSubCubeToStow() };
        }
        if (currentSetpoint == ArmSetpoints.GROUND_INTAKE_CUBE
                && (desiredSetpoint != ArmSetpoints.STOW && desiredSetpoint != ArmSetpoints.CUBE_STOW)) {
            arm.setCurrentSetpoint(ArmSetpoints.STOW);
            return new Trajectory[] { armTrajectories.getGroundIntakeCubeToStow() };
        }
        // if (currentSetpoint == ArmSetpoints.SINGLE_SUBSTATION
        // && (desiredSetpoint != ArmSetpoints.STOW && desiredSetpoint !=
        // ArmSetpoints.CUBE_STOW)) {
        // arm.setCurrentSetpoint(ArmSetpoints.STOW);
        // return new Trajectory[] { armTrajectories.getSingleSubToStow()};

        // }

        switch (desiredSetpoint) {
            case PLACE_TOP:
                leds.setLEDPattern(LEDPattern.BOUNCE);

                // if already there, return
                if (desiredSetpoint == currentSetpoint) {
                    leds.setLEDPattern(leds.getDefaultColor());
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getTopToStow() };
                } 
                // if at mid, run the node to node
                else if (currentSetpoint == ArmSetpoints.PLACE_MID) {
                    arm.setCurrentSetpoint(ArmSetpoints.PLACE_TOP);
                    return new Trajectory[] { armTrajectories.getMidToTop() };
                }
                // otherwise do it normally
                else {
                    arm.setCurrentSetpoint(ArmSetpoints.PLACE_TOP);
                    // if at sting, run the trajectory. If not at sting, go to sting then run the
                    // trajectory
                    if (currentSetpoint == ArmSetpoints.STOW) {
                        return new Trajectory[] { armTrajectories.getStowToTop() };
                    } else if (currentSetpoint == ArmSetpoints.CUBE_STOW) {
                        return new Trajectory[] { armTrajectories.getCubeStowToTop() };
                    }
                    System.out.println("Sting to top");
                    return new Trajectory[] { armTrajectories.getStingToTop() };
                }

            case PLACE_MID:
                leds.setLEDPattern(LEDPattern.BOUNCE);

                if (desiredSetpoint == currentSetpoint) {
                    leds.setLEDPattern(leds.getDefaultColor());
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getMidToStow() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_TOP) {
                    arm.setCurrentSetpoint(ArmSetpoints.PLACE_MID);
                    return new Trajectory[] { armTrajectories.getTopToMid() };
                } else {
                    if (currentSetpoint == ArmSetpoints.STING) {
                        arm.setCurrentSetpoint(ArmSetpoints.PLACE_MID);
                        return new Trajectory[] { armTrajectories.getStingToMid() };
                    } else if (currentSetpoint == ArmSetpoints.CUBE_STOW) {
                        arm.setCurrentSetpoint(ArmSetpoints.PLACE_MID);

                        return new Trajectory[] { armTrajectories.getCubeStowToMid() };
                    }
                    arm.setCurrentSetpoint(ArmSetpoints.PLACE_MID);
                    return new Trajectory[] { armTrajectories.getStowToMid() };

                }

            case DOUBLE_SUBSTATION_CONE:
                leds.setLEDPattern(LEDPattern.TWINKLE);

                if (desiredSetpoint == currentSetpoint) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getDoubleSubConeToStow() };
                } else if (desiredSetpoint == ArmSetpoints.CUBE_STOW) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getDoubleSubConeToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_TOP) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getTopToStow() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_MID) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getMidToStow() };
                } else if (currentSetpoint == ArmSetpoints.GROUND_INTAKE_CUBE) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getGroundIntakeCubeToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.STING) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getStingToStow() };
                } else if (currentSetpoint == ArmSetpoints.CUBE_STOW) {
                    arm.setCurrentSetpoint(ArmSetpoints.DOUBLE_SUBSTATION_CONE);
                    return new Trajectory[] { armTrajectories.getCubeStowToDoubleSubCone() };
                } else {
                    arm.setCurrentSetpoint(ArmSetpoints.DOUBLE_SUBSTATION_CONE);
                    return new Trajectory[] { armTrajectories.getStowToDoubleSubCone() };
                }
            case DOUBLE_SUBSTATION_CUBE:
                leds.setLEDPattern(LEDPattern.TWINKLE);

                if (desiredSetpoint == currentSetpoint) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getDoubleSubCubeToStow() };
                } else if (desiredSetpoint == ArmSetpoints.CUBE_STOW) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getDoubleSubCubeToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_TOP) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getTopToStow() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_MID) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getMidToStow() };
                } else if (currentSetpoint == ArmSetpoints.GROUND_INTAKE_CUBE) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getGroundIntakeCubeToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.STING) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getStingToStow() };
                } else if (currentSetpoint == ArmSetpoints.CUBE_STOW) {
                    arm.setCurrentSetpoint(ArmSetpoints.DOUBLE_SUBSTATION_CUBE);
                    return new Trajectory[] { armTrajectories.getCubeStowToDoubleSubCube() };
                } else {
                    arm.setCurrentSetpoint(ArmSetpoints.DOUBLE_SUBSTATION_CUBE);
                    return new Trajectory[] { armTrajectories.getStowToDoubleSubCube() };
                }

                // case SINGLE_SUBSTATION:
                // leds.setLEDPattern(LEDPattern.TWINKLE);

                // if (desiredSetpoint == currentSetpoint) {
                // arm.setCurrentSetpoint(ArmSetpoints.STOW);
                // return new Trajectory[] { armTrajectories.getSingleSubToStow() };
                // } else if (desiredSetpoint == ArmSetpoints.CUBE_STOW) {
                // arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                // return new Trajectory[] { armTrajectories.getSingleSubToCubeStow() };
                // } else if (currentSetpoint == ArmSetpoints.PLACE_TOP) {
                // arm.setCurrentSetpoint(ArmSetpoints.STOW);
                // return new Trajectory[] { armTrajectories.getTopToStow() };
                // } else if (currentSetpoint == ArmSetpoints.PLACE_MID) {
                // arm.setCurrentSetpoint(ArmSetpoints.STOW);
                // return new Trajectory[] { armTrajectories.getMidToStow() };
                // } else if (currentSetpoint == ArmSetpoints.GROUND_INTAKE_CUBE) {
                // arm.setCurrentSetpoint(ArmSetpoints.STOW);
                // return new Trajectory[] { armTrajectories.getGroundIntakeCubeToCubeStow() };
                // } else if (currentSetpoint == ArmSetpoints.STING) {
                // arm.setCurrentSetpoint(ArmSetpoints.STOW);
                // return new Trajectory[] { armTrajectories.getStingToStow() };
                // } else if (currentSetpoint == ArmSetpoints.CUBE_STOW) {
                // arm.setCurrentSetpoint(ArmSetpoints.SINGLE_SUBSTATION);
                // return new Trajectory[] { armTrajectories.getCubeStowToSingleSub() };
                // } else {
                // arm.setCurrentSetpoint(ArmSetpoints.SINGLE_SUBSTATION);
                // return new Trajectory[] { armTrajectories.getStowToSingleSub() };
                // }

            case GROUND_INTAKE_CUBE:
                leds.setLEDPattern(LEDPattern.WHITE);
                if (desiredSetpoint == currentSetpoint) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    leds.setLEDPattern(leds.getDefaultColor());
                    return new Trajectory[] { armTrajectories.getGroundIntakeCubeToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.CUBE_STOW) {
                    arm.setCurrentSetpoint(ArmSetpoints.GROUND_INTAKE_CUBE);
                    return new Trajectory[] { armTrajectories.getCubeStowToGroundIntakeCube() };
                } else {
                    arm.setCurrentSetpoint(ArmSetpoints.GROUND_INTAKE_CUBE);
                    return new Trajectory[] { armTrajectories.getStowToGroundIntakeCube() };
                }
            case STING:
                leds.setLEDPattern(LEDPattern.RAINBOW);

                if (desiredSetpoint == currentSetpoint && fromAlign) {
                    return null;
                } else if (desiredSetpoint == currentSetpoint) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    leds.setLEDPattern(leds.getDefaultColor());
                    return new Trajectory[] { armTrajectories.getStingToStow() };
                } else if (currentSetpoint == ArmSetpoints.CUBE_STOW) {
                    arm.setCurrentSetpoint(ArmSetpoints.STING);
                    return new Trajectory[] { armTrajectories.getCubeStowToSting() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_TOP) {
                    arm.setCurrentSetpoint(ArmSetpoints.STING);
                    return new Trajectory[] { armTrajectories.getTopToSting() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_MID) {
                    arm.setCurrentSetpoint(ArmSetpoints.STING);
                    return new Trajectory[] { armTrajectories.getMidToSting() };
                } else {
                    arm.setCurrentSetpoint(ArmSetpoints.STING);
                    return new Trajectory[] { armTrajectories.getStowToSting() };
                }
            case STOW:
                if (desiredSetpoint == currentSetpoint) {
                    return null;
                } else if (desiredSetpoint == ArmSetpoints.CUBE_STOW) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getStowToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_TOP) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getTopToStow() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_MID) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getMidToStow() };
                } else if (currentSetpoint == ArmSetpoints.DOUBLE_SUBSTATION_CONE) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getDoubleSubConeToStow() };
                } else if (currentSetpoint == ArmSetpoints.DOUBLE_SUBSTATION_CUBE) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getDoubleSubCubeToStow() };
                } else if (currentSetpoint == ArmSetpoints.GROUND_INTAKE_CUBE) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getGroundIntakeCubeToStow() };
                } else if (currentSetpoint == ArmSetpoints.GROUND_INTAKE_CUBE) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getGroundIntakeCubeToStow() };}else if (currentSetpoint == ArmSetpoints.STING) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getStingToStow() };
                }
            case CUBE_STOW:
                if (desiredSetpoint == currentSetpoint) {
                    arm.setCurrentSetpoint(ArmSetpoints.STOW);
                    return new Trajectory[] { armTrajectories.getCubeStowToStow() };
                } else if (currentSetpoint == ArmSetpoints.STING) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getStingToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_TOP) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getTopToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.PLACE_MID) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getMidToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.DOUBLE_SUBSTATION_CUBE) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getDoubleSubCubeToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.DOUBLE_SUBSTATION_CONE) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getDoubleSubConeToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.GROUND_INTAKE_CUBE) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getGroundIntakeCubeToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.STING) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getStingToCubeStow() };
                } else if (currentSetpoint == ArmSetpoints.STOW) {
                    arm.setCurrentSetpoint(ArmSetpoints.CUBE_STOW);
                    return new Trajectory[] { armTrajectories.getStowToCubeStow() };
                } else
                    return null;
            default:
                System.out.println(desiredSetpoint.name());
                return null;

        }
    }
}