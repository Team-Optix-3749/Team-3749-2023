package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Parent class for modular hardware
 * 
 * @author Rohin Sood
 * @author Raadwan Masum
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
	 *         perpendicular to the floor [0, 360]
	 */
	public double getShoulderAngle() {
		return 0.0;
	}

	/**
	 * sets the angle of the shoulder using a shoulder PIDController
	 */
	public void setShoulderAngle(double angle) {
	}

	/**
	 * @return true if the the shoulder PIDController.atSetpoint() returns true
	 *         the PIDController should have a tolerance
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
	 * @return the position of the elbow absolute encoder. the zero position is
	 *         parallel to the floor [0, 360]
	 */
	public double getElbowAngle() {
		return 0.0;
	}

	/**
	 * @return true if the the elbow PIDController.atSetpoint() returns true
	 *         the PIDController should have a tolerance
	 */
	public boolean getElbowAtSetpoint() {
		return false;
	}

	/**
	 * sets the angle of the elbow using a elbow PIDController
	 */
	public void setElbowAngle(double angle) {
	}

	/**
	 * sets the angle of the shoulder & elbow joints using the setShoulderAngle and
	 * setElbowAngle methods [0, 360]
	 * 
	 * @param shoulder_angle double
	 * @param elbow_angle    double
	 */
	public void setArmAngle(double shoulder_angle, double elbow_angle) {
	}

	/**
	 * sets the angle of the elbow and shoulder joints from a SendableChooser on
	 * SmartDashboard. resorts to the ELbowSetpoints and ShoulderSetpoints enums in
	 * Constants.java
	 * 
	 * @param shoulder_angle double
	 * @param elbow_angle    double
	 */
	public void setArmPreset() {
	}

	/**
	 * calls the stopShoulder() and stopElbow() methods
	 */
	public void stop() {
	}

	/**
	 * sets the control mode of all motor controllers
	 * 
	 * @param idleMode IdleMode.kBrake or IdleMode.kCoast static references
	 */
	public void setIdleMode(IdleMode idleMode) {
	}

    /**
	 * sets the tolerance of the arm PID controllers
	 */
    public void setArmTolerance(double tolerance) {

    }

}
