package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.arm.*;
import frc.robot.commands.*;
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.Constants.*;

public class RobotContainer {
    // Controllers
    private final Xbox pilot = new Xbox(OIConstants.kPilotControllerPort);
    // private final Xbox operator = new Xbox(OIConstants.kOperatorControllerPort);

    // Subsystems
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Claw claw = new Claw();
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

        setupAuto();
        swerveSubsystem.setDefaultCommand(new SwerveTeleopCommand(
                swerveSubsystem,
                () -> -pilot.getLeftY(),
                () -> pilot.getLeftX(),
                () -> pilot.getRightX()));

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
                        Commands.run(() -> arm.setIdleMode(IdleMode.kBrake), arm));
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
                pilot.xWhileHeld(() -> swerveSubsystem.zeroHeading(), swerveSubsystem);

                pilot.rightBumperWhileHeld(() -> claw.set(-Constants.Claw.speed.get()),
                        () -> claw.set(Constants.Claw.speed.get()), claw);

                pilot.y().whileTrue(new SequentialCommandGroup(
                        new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                                Constants.Arm.ElbowSetpoints.STING.angle),
                        new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.CONE_TOP.angle,
                                Constants.Arm.ElbowSetpoints.CONE_TOP.angle)));

                pilot.b().whileTrue(new SequentialCommandGroup(
                        new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                                Constants.Arm.ElbowSetpoints.STING.angle),
                        new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.CONE_MID.angle,
                                Constants.Arm.ElbowSetpoints.CONE_MID.angle)));

                pilot.a().whileTrue(new SequentialCommandGroup(
                        // new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                        //         Constants.Arm.ElbowSetpoints.STING.angle),
                        new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.DOUBLE_SUBSTATION.angle,
                                Constants.Arm.ElbowSetpoints.DOUBLE_SUBSTATION.angle)));

                pilot.start().whileTrue(new SequentialCommandGroup(
                        new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                                Constants.Arm.ElbowSetpoints.STING.angle),
                        new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.GROUND_INTAKE.angle,
                                Constants.Arm.ElbowSetpoints.GROUND_INTAKE.angle)));

                pilot.leftBumper().whileTrue(new SequentialCommandGroup(
                        new MoveArmPID(arm, Constants.Arm.ShoulderSetpoints.STING.angle,
                                Constants.Arm.ElbowSetpoints.STING.angle),
                        new MoveArmHoldPID(arm, Constants.Arm.ShoulderSetpoints.STOWED.angle,
                                Constants.Arm.ElbowSetpoints.STOWED.angle)));
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
        return AutoCommands.getTestPathPlanner(swerveSubsystem, Alliance.Blue);
    }

    /**
     * Set event maps for autonomous
     */
    public void setupAuto() {
        AutoConstants.eventMap.put("pickup_cone_floor", Commands.print("PICKUP CONE FLOOR"));
        AutoConstants.eventMap.put("pickup_cube_floor", null);
        AutoConstants.eventMap.put("pickup_cone_double_substation", null);
        AutoConstants.eventMap.put("pickup_cube_double_substation", null);
        AutoConstants.eventMap.put("pickup_cone_single_substation", null);
        AutoConstants.eventMap.put("pickup_cube_single_substation", null);
        AutoConstants.eventMap.put("place_cone_bottom", null);
        AutoConstants.eventMap.put("place_cube_bottom", null);
        AutoConstants.eventMap.put("place_cone_mid", null);
        AutoConstants.eventMap.put("place_cube_mid", null);
        AutoConstants.eventMap.put("place_cone_top", null);
        AutoConstants.eventMap.put("place_cube_top", null);
        // Constants.AutoConstants.eventMap.put("run_claw", Commands.run(() ->
        // clawSubsystem.set(0.2), clawSubsystem));
    }
}