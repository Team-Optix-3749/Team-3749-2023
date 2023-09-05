package frc.robot.subsystems.swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.Sim;

/* 
Very closely inspired by 6328's Swerve Sim code,
 https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/drive/ModuleIOSim.java
*/
public class SwerveModuleSim implements SwerveModuleIO {
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public SwerveModuleSim() {
        System.out.println("[Init] Creating ModuleIOSim");
    }

    public void updateInputs(ModuleData data) {
        // update sim values
        driveSim.update(Sim.loopPeriodSec);
        turnSim.update(Sim.loopPeriodSec);

        // how far have we turned in the previous loop?
        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Sim.loopPeriodSec;
        // update our angle variables
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        // keep our absolute position within 0-2 pi
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }
        // distance traveled + Rad/Time * Time * diameter
        data.drivePositionM = data.drivePositionM + (driveSim.getAngularVelocityRadPerSec() * 0.02 * ModuleConstants.kWheelDiameterMeters);
        data.driveVelocityMPerSec = driveSim.getAngularVelocityRadPerSec() * ModuleConstants.kWheelDiameterMeters;
        
    }
}