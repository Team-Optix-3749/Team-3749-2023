package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;

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

	public void stopShoulder() {
	}

	public void setElbow(double percent) {
	}

	public void stopElbow() {
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

	public boolean getShoulderAtSetpoint(double angle) {
		return false;
	}

	public void setShoulderAngleWaypoints(double angle1, double angle2) {
	}

	public void setShoulderAngle(double angle) {
	}

	public boolean isShoulderAtSetpoint() {
		return false;
	}

	public boolean isElbowAtSetpoint() {
		return false;
	}

	public void setElbowAngle(double angle) {
	}

	public void setArmAngle(double shoulder_angle, double elbow_angle) {
	}

	public void setArmPreset() {
	}

	public void stop() {
	}

	public void setIdleMode(IdleMode idleMode) {
	}

}
