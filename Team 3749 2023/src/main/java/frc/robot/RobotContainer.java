package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.arm.*;
import frc.robot.commands.*;
import frc.robot.utils.*;
import frc.robot.utils.Constants;

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
                arm.setDefaultCommand(
                        new ArmTeleopCommand(arm, pilot));
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
                // pilot.y().whileTrue(new SequentialCommandGroup(
                //         new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                //                 Constants.Arm.ElbowSetpoints.STING.angle),
                //         new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.CONE_TOP.angle,
                //                 Constants.Arm.ElbowSetpoints.CONE_TOP.angle)));

                // pilot.b().whileTrue(new SequentialCommandGroup(
                //         new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                //                 Constants.Arm.ElbowSetpoints.STING.angle),
                //         new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.CONE_MID.angle,
                //                 Constants.Arm.ElbowSetpoints.CONE_MID.angle)));

                // pilot.a().whileTrue(new SequentialCommandGroup(
                //         // new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                //         //         Constants.Arm.ElbowSetpoints.STING.angle),
                //         new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.DOUBLE_SUBSTATION.angle,
                //                 Constants.Arm.ElbowSetpoints.DOUBLE_SUBSTATION.angle)));

                // pilot.start().whileTrue(new SequentialCommandGroup(
                //         new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                //                 Constants.Arm.ElbowSetpoints.STING.angle),
                //         new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.TOP_INTAKE.angle,
                //                 Constants.Arm.ElbowSetpoints.TOP_INTAKE.angle)));

                // pilot.leftBumper().whileTrue(new SequentialCommandGroup(
                //         new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                //                 Constants.Arm.ElbowSetpoints.STING.angle),
                //         new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.STOWED.angle,
                //                 Constants.Arm.ElbowSetpoints.STOWED.angle)));
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