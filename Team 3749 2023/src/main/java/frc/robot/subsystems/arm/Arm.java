package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Parent class for modular hardware
 * 
 * @author Rohin Sood
 */
public class Arm extends SubsystemBase {

	public void setElbowVoltage(double voltage) {
	}

	public void setShoulderVoltage(double voltage) {
	}

	public void setShoulder(double percent) {
	}

	public void setElbow(double percent) {
	}

	public double getShoulderDistance() {
		return 0.0;
	}

	public double getElbowDistance() {
		return 0.0;
	}

	public void setShoulderPosition(double position) {
	}

	public void setElbowPosition(double position) {
	}

	public void setShoulderAngle(double angle) {}
	
	public void setElbowAngle(double angle) {}

	public void setArmAngle(double shoulder_angle, double elbow_angle) {}
	
	public void setArmPreset() {}

	public void stop() {}

}