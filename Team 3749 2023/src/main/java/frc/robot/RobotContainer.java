package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.arm.*;
import frc.robot.commands.*;
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

public class RobotContainer {
    // Controllers
    private final Xbox pilot = new Xbox(0);

    // Subsystems
    private final Arm arm;

    public RobotContainer() {
        switch (Constants.ROBOT_MODE) {
            case REAL:
                arm = new ArmReal();
                break;
            case SIMULATION:
                arm = new ArmSim();
                break;
            default:
                arm = null;
                System.out.println("ROBOT_MODE is not set in utils/Constants.java");
                break;
        }

        try {
            configureDefaultCommands();
            configureButtonBindings();
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    /**
     * Set default commands
     * 
     * @throws Exception
     */
    private void configureDefaultCommands() throws Exception {
        switch (Constants.ROBOT_MODE) {
            case REAL:
                arm.setDefaultCommand(new ArmTeleopCommand(arm));
                break;
            case SIMULATION:
                arm.setDefaultCommand(
                        new ArmSimCommand(arm));
            default:
                throw new Exception("ROBOT_MODE is not set in utils/Constants.java");
        }
    }

    /**
     * Set controller button bindings
     * 
     * @throws Exception
     */
    private void configureButtonBindings() throws Exception {

        switch (Constants.ROBOT_MODE) {
            case REAL:
                pilot.aWhileHeld(
                        () -> {
                            Constants.desired_setpoint = ArmSetpoints.DOUBLE_SUBSTATION;
                            System.out.println(Constants.desired_setpoint);

                        });

                pilot.bWhileHeld(
                        () -> {
                            Constants.desired_setpoint = ArmSetpoints.STOWED;
                            System.out.println(Constants.desired_setpoint);

                        });

                pilot.xWhileHeld(
                        () -> {
                            Constants.desired_setpoint = ArmSetpoints.CONE_MID;
                            System.out.println(Constants.desired_setpoint);

                        });

                pilot.yWhileHeld(
                        () -> {
                            Constants.desired_setpoint = ArmSetpoints.CONE_TOP;
                            System.out.println(Constants.desired_setpoint);

                        });

                pilot.rightBumperWhileHeld(
                        () -> {
                            Constants.desired_setpoint = ArmSetpoints.STING;
                            System.out.println(Constants.desired_setpoint);

                        });

                pilot.startWhileHeld(
                        () -> {
                            Constants.desired_setpoint = ArmSetpoints.TOP_INTAKE;
                            System.out.println(Constants.desired_setpoint);
                        });
                break;
            case SIMULATION:
                break;
            default:
                throw new Exception("ROBOT_MODE is not set in utils/Constants.java");
        }
    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        return new PrintCommand("No Auto");
    }
}