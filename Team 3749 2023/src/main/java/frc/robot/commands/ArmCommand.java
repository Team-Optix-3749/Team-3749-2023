// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Don Tran
 * @author Bailey Say
 * @author Raymond Sheng
 * 
 */
public class ArmCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private Arm arm;

    // Initializes the ArmCommand
    public ArmCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // temporary: set voltages to move to set point (need to implement enum w/ it)
    // relative encoder using getPosition() instead of getDistance()
    // make sure conversion factor is good
    public void goToSetPoint(double elbowSetpoint, double shoulderSetpoint){
        // put to SmartDashboard
        SmartDashboard.putNumber("Setpoint bottom (degrees)", shoulderSetpoint);
        SmartDashboard.putNumber("Setpoint top (degrees)", elbowSetpoint);
        
        // Here, we run PID control where the arm moves to the selected setpoint.
        double pidOutputElbow = arm.getElbowController().calculate(arm.getElbowEncoder().getPosition(),
            Units.degreesToRadians(elbowSetpoint - shoulderSetpoint));
        arm.setElbowVoltage(pidOutputElbow);

        double pidOutputShoulder = arm.getShoulderController().calculate(arm.getShoulderEncoder().getPosition(),
            Units.degreesToRadians(shoulderSetpoint));
        arm.setShoulderVoltage(pidOutputShoulder);
    }

    // Run on command init
    @Override
    public void initialize() {}

    // Run every 20 ms
    @Override
    public void execute() { // we will figure this out later
        goToSetPoint(0, 0);
    }

    // Run on command finish
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}