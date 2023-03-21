package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    
    @AutoLog
    public static class ArmIOInputs {
        public double x = 0.0;
        public double y = 0.0;
        public double elbowVoltage = 0.0;
        public double shoulderVoltage = 0.0;
        public double elbowAngle = 0.0;
        public double shoulderAngle = 0.0;
    }
}
