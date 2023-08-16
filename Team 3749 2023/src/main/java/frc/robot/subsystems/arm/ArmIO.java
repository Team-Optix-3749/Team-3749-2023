package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

public interface ArmIO {
    public static class ArmData {
        public Mechanism2d armMechanism = new Mechanism2d(
                (Constants.Arm.elbow_length + Constants.Arm.shoulder_length) * 2,
                (Constants.Arm.elbow_length + Constants.Arm.shoulder_length) * 2);
        public MechanismRoot2d armMechanismRoot = armMechanism.getRoot("Shoulder Pivot",
                Constants.Arm.elbow_length + Constants.Arm.shoulder_length,
                Constants.Arm.elbow_length + Constants.Arm.shoulder_length);
        public MechanismLigament2d shoulderLigament;
        public MechanismLigament2d elbowLigament;

        public Translation2d setpoint = ArmSetpoints.STOW.translation;
        public Translation2d position = ArmSetpoints.STOW.translation;
        public ArmSetpoints currentSetpoint = ArmSetpoints.STOW;

        public double shoulderVoltage = 0.0;
        public double shoulderFF = 0.0;
        public double shoulderPID = 0.0;
        public double shoulderAngle = 0.0;
        public double shoulderTemp = 0.0;
        public double elbowVoltage = 0.0;
        public double elbowFF = 0.0;
        public double elbowPID = 0.0;
        public double elbowAngle = 0.0;
        public double elbowTemp = 0.0;

        public boolean kill = false;

    }

    public default void updateData(ArmData data) {
    }



    public default void moveArm(Translation2d setpoint, ArmDynamics dynamics) throws Exception {
    }


    public default void setArmBrakeMode(boolean enable) {
    }

    public default void periodic() {
    }

}
