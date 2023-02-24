package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/**
 * Double jointed arm subsystem built with 2 CANSparkMaxes at each joint and REV
 * Through Bore Encoders
 * 
 * TODO: increase tolerance on sting, change to brake when at setpoint
 * 
 * @author Rohin Sood
 */
public class Arm extends SubsystemBase {

    private final ArmDynamics dynamics = new ArmDynamics();
    private final ArmKinematics kinematics = new ArmKinematics();

    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);
    private final DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(0);
    private final PIDController shoulderPIDController = new PIDController(Constants.Arm.shoulder_kP.get(), 0, 0);

    private final CANSparkMax elbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(2);
    private final PIDController elbowPIDController = new PIDController(Constants.Arm.elbow_kP.get(), 0, 0);

    private final SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

    public Arm() {
        shoulderMotor.restoreFactoryDefaults();
        elbowMotor.restoreFactoryDefaults();
        // elbow offset is done in the get angle method
        shoulderAbsoluteEncoder.setPositionOffset(Constants.Arm.shoulder_offset);
        shoulderAbsoluteEncoder.setDistancePerRotation(360);
        elbowAbsoluteEncoder.setDistancePerRotation(360);

        elbowMotor.setInverted(true);
        shoulderMotor.setInverted(false);

        presetChooser.setDefaultOption("Stowed", 0);
        presetChooser.addOption("DS", 1);
        SmartDashboard.putData(presetChooser);

        shoulderPIDController.setTolerance(0);
        elbowPIDController.setTolerance(0);

        shoulderMotor.setIdleMode(IdleMode.kCoast);
        elbowMotor.setIdleMode(IdleMode.kCoast);
    }

    // public void setArmPosition(double shoulderAngle, double elbowAngle) {
    //     double[] feedForwardOutput = dynamics.feedforward(VecBuilder.fill(shoulderAngle, elbowAngle * -1)).getData();

    //     setShoulderVoltage(shoulderPIDController.calculate(getShoulderAngle(), shoulderAngle) + feedForwardOutput[0]);
    //     setElbowVoltage(-elbowPIDController.calculate(getElbowAngle(), elbowAngle) + feedForwardOutput[1]);

    //     SmartDashboard.putNumber("SHOULDER FF",
    //             dynamics.feedforward(VecBuilder.fill(shoulderAngle, elbowAngle)).getData()[0]);
    //     SmartDashboard.putNumber("ELBOW FF",
    //             dynamics.feedforward(VecBuilder.fill(shoulderAngle, elbowAngle)).getData()[1]);
    //     SmartDashboard.putNumberArray("SIMULATION FF",
    //             dynamics.feedforward(VecBuilder.fill(shoulderAngle, elbowAngle)).getData());

    //     SmartDashboard.putNumber("shoulder pid", shoulderPIDController.calculate(getShoulderAngle(), shoulderAngle));
    //     SmartDashboard.putNumber("elbow pid", elbowPIDController.calculate(getElbowAngle(), elbowAngle));
    // }

    public void setArmPosition(Translation2d position){
        System.out.println("hello");
        
        double shoulderAngle;
        double elbowAngle;

        try {
            shoulderAngle = kinematics.inverse(position.getX(), position.getY()).getFirst();
            elbowAngle = kinematics.inverse(position.getX(), position.getY()).getSecond();
        } catch (Exception e) {
            shoulderAngle = 90;
            elbowAngle = -90;
            System.out.println(e);
        }

        // double shoulderAngle = kinematics.inverse(getArmCoordinate().getX(), getArmCoordinate().getY()).getFirst();
        // double elbowAngle = kinematics.inverse(getArmCoordinate().getX(), getArmCoordinate().getY()).getSecond();

        double[] feedForwardOutput = dynamics.feedforward(VecBuilder.fill(shoulderAngle, elbowAngle)).getData();

        // setShoulderVoltage(shoulderPIDController.calculate(getShoulderAngle(), shoulderAngle) + feedForwardOutput[0]);
        // setElbowVoltage(elbowPIDController.calculate(getElbowAngle(), elbowAngle) + feedForwardOutput[1]);

        // setShoulderVoltage(feedForwardOutput[0]);
        // setElbowVoltage(feedForwardOutput[1]);

        SmartDashboard.putNumber("SA SETPOINT", shoulderAngle);
        SmartDashboard.putNumber("EA SETPOINT", elbowAngle);

        // SmartDashboard.putNumber("SHOULDER FF", feedForwardOutput[0]);
        // SmartDashboard.putNumber("ELBOW FF", feedForwardOutput[1]);

        SmartDashboard.putNumber("SHOULDER PID", shoulderPIDController.calculate(getShoulderAngle(), shoulderAngle) + feedForwardOutput[0]);
        SmartDashboard.putNumber("ELBOW PID", elbowPIDController.calculate(getElbowAngle(), elbowAngle) + feedForwardOutput[1]);
    }

    public Translation2d getArmCoordinate(){
        return kinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle()));
    }

    public void setShoulderVoltage(double voltage) {
        shoulderMotor.setVoltage(voltage);
    }

    public void setElbowVoltage(double voltage) {
        elbowMotor.setVoltage(voltage);
    }

    public void setShoulder(double percent) {
        boolean past_min_limit = getShoulderAngle() <= Constants.Arm.shoulder_min_angle && percent > 0;
        boolean past_max_limit = getShoulderAngle() >= Constants.Arm.shoulder_max_angle && percent < 0;
        if (past_min_limit || past_max_limit) {
            return;
        }
        shoulderMotor.set(percent);
    }

    public void setElbow(double percent) {
        elbowMotor.set(percent);
    }

    public double getShoulderAngle() {
        return shoulderAbsoluteEncoder.getDistance();
    }

    public double getElbowAngle() {
        return new Rotation2d(Math.toRadians(elbowAbsoluteEncoder.getAbsolutePosition() * 360 - 180))
        .rotateBy(new Rotation2d(Math.toRadians(180))).getDegrees() - Constants.Arm.elbow_offset;
    }

    public void setShoulderAngle(double angle) {
        shoulderPIDController.setSetpoint(angle);

        shoulderMotor.set(
                shoulderPIDController.calculate(
                        shoulderAbsoluteEncoder.getDistance()));
    }

    public void setElbowAngle(double angle) {
        elbowPIDController.setSetpoint(angle);

        elbowMotor.set(
                elbowPIDController.calculate(
                        getElbowAngle()));
    }

    public boolean getShoulderAtSetpoint() {
        return shoulderPIDController.atSetpoint();
    }

    public boolean getElbowAtSetpoint() {
        return elbowPIDController.atSetpoint();
    }

    public void setArmAngle(double shoulder_angle, double elbow_angle) {
        setShoulderAngle(shoulder_angle);
        setElbowAngle(elbow_angle);
    }


    public void setArmPreset() {
        double shoulder_angle;
        double elbow_angle;

        switch (presetChooser.getSelected()) {
            case (0):
                shoulder_angle = Constants.Arm.ShoulderSetpoints.STOWED.angle;
                elbow_angle = Constants.Arm.ElbowSetpoints.STOWED.angle;
                break;
            case (1):
                shoulder_angle = Constants.Arm.ShoulderSetpoints.DOUBLE_SUBSTATION.angle;
                elbow_angle = Constants.Arm.ElbowSetpoints.DOUBLE_SUBSTATION.angle;
                break;
            default:
                shoulder_angle = Constants.Arm.ShoulderSetpoints.STOWED.angle;
                elbow_angle = Constants.Arm.ElbowSetpoints.STOWED.angle;
                break;
        }

        setShoulderAngle(shoulder_angle);
        setElbowAngle(elbow_angle);
    }

    public void stop() {
        elbowMotor.stopMotor();
        shoulderMotor.stopMotor();
    }

    public void stopShoulder() {
        shoulderMotor.stopMotor();
    }

    public void stopElbow() {
        elbowMotor.stopMotor();
    }

    public void periodic() {
        // SmartDashboard.putNumber("Shoulder Right Amps", shoulderMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Elbow right Amps", elbowMotor.getOutputCurrent());

        SmartDashboard.putNumber("shoulder angle", shoulderAbsoluteEncoder.getDistance());
        SmartDashboard.putNumber("elbow angle", getElbowAngle());

        SmartDashboard.putNumber("shoulder raw pos", shoulderAbsoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("elbow raw pos", elbowAbsoluteEncoder.getAbsolutePosition());

        // setArmPosition(90, 90);

        // setArmPosition(new Translation2d(1.3, 0.8));

        shoulderPIDController.setP(Constants.Arm.shoulder_kP.get());
        elbowPIDController.setP(Constants.Arm.elbow_kP.get());

        // setShoulderVoltage(dynamics.feedforward(VecBuilder.fill(getShoulderAngle(), getElbowAngle())).getData()[0]);
        // setElbowVoltage(dynamics.feedforward(VecBuilder.fill(getShoulderAngle(), getElbowAngle())).getData()[1]);

        SmartDashboard.putNumber("SHOULDER FF", dynamics.feedforward(VecBuilder.fill(getShoulderAngle(), getElbowAngle())).getData()[0]);
        SmartDashboard.putNumber("ELBOW FF", dynamics.feedforward(VecBuilder.fill(getShoulderAngle(), getElbowAngle())).getData()[1]);

        // setArmPosition(kinematics.inverse(1.0, 0.6).getFirst(), kinematics.inverse(1.0, 0.6).getSecond());

        SmartDashboard.putNumber("ARM X",
                kinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle())).getX());
        SmartDashboard.putNumber("ARM Y",
                kinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle())).getY());
        
        try { 
            SmartDashboard.putNumber("ARM SA SETPOINT",
                    kinematics.inverse(kinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle())).getX(),
                    kinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle())).getY()).getFirst());
            SmartDashboard.putNumber("ARM EA SETPOINT",
                    kinematics.inverse(kinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle())).getX(), 
                    kinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle())).getY()).getSecond());
        } catch (Exception e) {
            System.out.println(e);
        }


        // SmartDashboard.putNumber(, getElbowAngle())

    }


}
