package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.claw.*;
import frc.robot.commands.*;
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

public class RobotContainer {
    // Controllers
    private final Xbox pilot = new Xbox(0);

    // Subsystems
    private final Swerve swerve = new Swerve();
    private final Claw claw = new Claw();
    private final Arm arm = new Arm();

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAuto();
    }

    /**
     * Set default commands
     * 
     */
    private void configureDefaultCommands() {
        arm.setDefaultCommand(new ArmTeleopCommand(arm));

        swerve.setDefaultCommand(new SwerveTeleopCommand(
                swerve,
                () -> -pilot.getLeftY(),
                () -> pilot.getLeftX(),
                () -> pilot.getRightX()));
    }

    /**
     * Set controller button bindings
     * 
     */
    private void configureButtonBindings() {
        pilot.aWhileHeld(() -> {
            Constants.desired_setpoint = ArmSetpoints.DOUBLE_SUBSTATION;
        });

        pilot.bWhileHeld(() -> {
            Constants.desired_setpoint = ArmSetpoints.STOWED;
        });

        pilot.xWhileHeld(() -> {
            Constants.desired_setpoint = ArmSetpoints.CONE_MID;
        });

        pilot.yWhileHeld(() -> {
            Constants.desired_setpoint = ArmSetpoints.CONE_TOP;
        });

        pilot.rightBumperWhileHeld(() -> {
            Constants.desired_setpoint = ArmSetpoints.STING;
        });

        pilot.startWhileHeld(() -> {
            Constants.desired_setpoint = ArmSetpoints.TOP_INTAKE;
        });

        pilot.backWhileHeld(() -> swerve.zeroHeading(), swerve);

        pilot.rightTriggerWhileHeld(() -> 
            claw.set(1));
        pilot.leftTriggerWhileHeld(() -> 
            claw.set(-0.125));
    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        return AutoCommands.getTestPathPlanner(swerve, Alliance.Blue);
    }

    /**
     * Set event maps for autonomous
     */
    public void configureAuto() {
        Constants.AutoConstants.eventMap.put("pickup_cone_floor", Commands.print("PICKUP CONE FLOOR"));
        Constants.AutoConstants.eventMap.put("pickup_cube_floor", null);
        Constants.AutoConstants.eventMap.put("pickup_cone_double_substation", null);
        Constants.AutoConstants.eventMap.put("pickup_cube_double_substation", null);
        Constants.AutoConstants.eventMap.put("pickup_cone_single_substation", null);
        Constants.AutoConstants.eventMap.put("pickup_cube_single_substation", null);
        Constants.AutoConstants.eventMap.put("place_cone_bottom", null);
        Constants.AutoConstants.eventMap.put("place_cube_bottom", null);
        Constants.AutoConstants.eventMap.put("place_cone_mid", null);
        Constants.AutoConstants.eventMap.put("place_cube_mid", null);
        Constants.AutoConstants.eventMap.put("place_cone_top", null);
        Constants.AutoConstants.eventMap.put("place_cube_top", null);
        // Constants.AutoConstants.eventMap.put("run_claw", Commands.run(() ->
        // claw.set(0.2), claw));
    }
}