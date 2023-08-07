package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

/**
 * Double jointed arm subsystem built with 2 CANSparkMaxes at each joint and REV
 * Through Bore Encoders
 * 
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 */

public class Arm extends SubsystemBase {

    public static final ArmTrajectories armTrajectories = new ArmTrajectories();

    private final ArmDynamics dynamics = new ArmDynamics();

    private ArmIO armIO;
    private ArmData armData = new ArmData();

    public Arm() {
        armIO = new ArmSparkMax();
        armIO.setArmBrakeMode(false);
        armData.shoulderLigament = armData.armMechanismRoot
                .append(new MechanismLigament2d("Shoulder", Constants.Arm.shoulder_length, getShoulderAngle()));
        armData.elbowLigament = armData.armMechanismRoot
                .append(new MechanismLigament2d("Elbow", Constants.Arm.shoulder_length, getElbowAngle()));

    }

    /**
     * Get current shoulder angle
     * 
     * @return shoulder angle as double
     */
    private double getShoulderAngle() {
        return armData.shoulderAngle;
    }

    /**
     * Get current elbow angle
     * 
     * @return elbow angle as double
     */
    private double getElbowAngle() {
        return armData.shoulderAngle;
    }

    public void setArmPosition(Translation2d pos) {
        if (armData.kill){
            return;
        }
        armData.setpoint = pos;
    }


    public void setCurrentSetpoint(ArmSetpoints setpoint) {
        if (armData.kill){
            return;
        }
        armIO.setCurrentSetpoint(setpoint);
    }

    public ArmSetpoints getCurrentSetpoint() {
        return armData.currentSetpoint;
    }
    public Translation2d getArmCoordinate(){
        return armData.position;
    }

    public void kill() {
        armData.kill = !armData.kill;

        if (armData.kill) {
            setArmBrakeMode(true);
            System.out.println("KILLED ARM");
        } else {
            setArmBrakeMode(false);
            System.out.println("UNKILLED ARM");
        }    }

    /**
     * @param mode IdleMode of the arm
     */
    public void setArmBrakeMode(boolean enable) {
        if (armData.kill){
            return;
        }
        armIO.setArmBrakeMode(enable);
    }

    @Override
    public void periodic() {
        try {
            if (armData.kill){
                return;
            }
            armIO.moveArm(armData.setpoint, dynamics);
        } catch (Exception e) {
            System.out.println(e);
        }

        armIO.updateData(armData);

        // shoulderAngle.set(getShoulderAngle());
        // elbowAngle.set(getElbowAngle());
        // shoulderLigament.setAngle(getShoulderAngle());
        // elbowLigament.setAngle(getElbowAngle());

        // SmartDashboard.putData("Arm Mechanism", armMechanism);

        // armCacheX.set(position.getX());
        // armCacheY.set(position.getY());

        // var kinematicsOutput =
        // ArmKinematics.forward(Math.toRadians(getShoulderAngle()),
        // Math.toRadians(getElbowAngle()));

        // armX.set(kinematicsOutput.getX());
        // armY.set(kinematicsOutput.getY());

        // shoulderVoltage.set(shoulderMotor.getBusVoltage() *
        // shoulderMotor.getAppliedOutput());
        // elbowVoltage.set(elbowMotor.getBusVoltage() * elbowMotor.getAppliedOutput());
    }
}