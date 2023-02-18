package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Parent class for modular hardware
 * 
 * @author Rohin Sood
 */
public class Arm extends SubsystemBase {

	/**
	 * @param voltage double
	 */
	public void setShoulderVoltage(double voltage) {
	}

	/**
	 * @param percent double
	 */
	public void setShoulder(double percent) {
	}

	/**
	 * @return the position of the shoulder absolute encoder. the zero position is
	 *         perpendicular to the floor
	 */
	public double getShoulderAngle() {
		return 0.0;
	}

	/**
	 * sets the angle of the shoulder using PIDControllers
	 */
	public void setShoulderAngle(double angle) {
	}

	/**
	 * returns the PIDController.atSetpoint() output
	 * the PIDController should have a tolerance
	 */
	public boolean getShoulderAtSetpoint() {
		return false;
	}

	/**
	 * runs .stopMotor() of the motor controller objects
	 */
	public void stopShoulder() {
	}

	/**
	 * @param voltage double
	 */
	public void setElbowVoltage(double voltage) {
	}

	/**
	 * @param voltage double
	 */
	public void setElbow(double percent) {
	}

	/**
	 * runs .stopMotor() of the motor controller objects
	 */
	public void stopElbow() {
	}

	/**
	 * @return the position of the shoulder absolute encoder. the zero position is
	 *         parallel to the floor
	 */
	public double getElbowAngle() {
		return 0.0;
	}

	public boolean getElbowAtSetpoint() {
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
