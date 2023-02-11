package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

	public void setElbowVoltage(double voltage) {}
	
	public void setShoulderVoltage(double voltage) {}

	public void setShoulder(double percent) {}

	public void setElbow(double percent) {}

	public double getShoulderPosition() { return 0.0; }

	public double getElbowPosition() { return 0.0; }

	public void setShoulderPosition(double position) {}

	public void setElbowPosition(double position) {}

}
