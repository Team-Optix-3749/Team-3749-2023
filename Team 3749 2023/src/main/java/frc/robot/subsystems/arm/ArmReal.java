package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;

/**
 * Double jointed arm subsystem built with 2 CANSparkMaxes at each joint and REV
 * Through Bore Encoders
 * 
 * @author Rohin Sood
 */
public class ArmReal extends Arm {

  private final CANSparkMax leftShoulderMotor = new CANSparkMax(Constants.Arm.left_shoulder_id, MotorType.kBrushless);
  private final RelativeEncoder leftShoulderRelativeEncoder = leftShoulderMotor.getEncoder();
  private final CANSparkMax rightShoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);
  private final RelativeEncoder rightShoulderRelativeEncoder = rightShoulderMotor.getEncoder();
  private final DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(0);
  private final PIDController shoulderPIDController = new PIDController(Constants.Arm.shoulderKP.get(), 0, 0);

  private final CANSparkMax leftElbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
  private final RelativeEncoder leftElbowRelativeEncoder = leftElbowMotor.getEncoder();
  private final CANSparkMax rightElbowMotor = new CANSparkMax(Constants.Arm.right_elbow_id, MotorType.kBrushless);
  private final RelativeEncoder rightElbowRelativeEncoder = rightElbowMotor.getEncoder();
  private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(1);
  private final PIDController elbowPIDController = new PIDController(Constants.Arm.elbowKP.get(), 0, 0);

  private final SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

  public ArmReal() {
    leftShoulderMotor.restoreFactoryDefaults();
    rightShoulderMotor.restoreFactoryDefaults();
    leftElbowMotor.restoreFactoryDefaults();
    rightElbowMotor.restoreFactoryDefaults();
    
    shoulderAbsoluteEncoder.setPositionOffset(Constants.Arm.shoulder_offset);
    elbowAbsoluteEncoder.setPositionOffset(Constants.Arm.elbow_offset);
    shoulderAbsoluteEncoder.setDistancePerRotation(360);
    elbowAbsoluteEncoder.setDistancePerRotation(360);
    
    leftShoulderMotor.setInverted(true);
    leftElbowMotor.setInverted(true);

    presetChooser.setDefaultOption("Stowed", 0);
    presetChooser.addOption("DS", 1);
    SmartDashboard.putData(presetChooser);
  }

  @Override
  public void setShoulder(double percent) {
    boolean past_min_limit = getShoulderDistance() <= Constants.Arm.shoulder_min_angle && percent < 0; 
    boolean past_max_limit = getShoulderDistance() >= Constants.Arm.shoulder_max_angle && percent > 0; 
    if (past_min_limit || past_max_limit) {
      return;
    }
    leftShoulderMotor.set(percent);
    rightShoulderMotor.set(percent);
  }


  @Override
  public void setElbow(double percent) {
    leftElbowMotor.set(percent);
    rightElbowMotor.set(percent);
  }

  @Override
  public double getShoulderDistance() {
    return shoulderAbsoluteEncoder.getDistance();
  }

  @Override
  public double getElbowDistance() {
    return elbowAbsoluteEncoder.getDistance();
  }

  @Override
  public void setShoulderAngle(double angle) {
    leftShoulderMotor.set(
      shoulderPIDController.calculate(
        shoulderAbsoluteEncoder.getDistance(), angle)
    );

    rightShoulderMotor.set(
      shoulderPIDController.calculate(
        shoulderAbsoluteEncoder.getDistance(), angle)
    );
  }

  @Override
  public void setElbowAngle(double angle) {
    leftElbowMotor.set(
      elbowPIDController.calculate(
        elbowAbsoluteEncoder.getDistance(), angle)
    );

    rightElbowMotor.set(
      elbowPIDController.calculate(
        elbowAbsoluteEncoder.getDistance(), angle)
    );
  }

  @Override
  public void setArmAngle(double shoulder_angle, double elbow_angle) {
    setShoulderAngle(shoulder_angle);
    setElbowAngle(elbow_angle);
  }

  @Override
  public void setArmPreset() {
    double shoulder_angle;
    double elbow_angle;

    switch(presetChooser.getSelected()) {
      case (0):
        shoulder_angle = Constants.Arm.ShoulderSetpoints.STOWED.angle;
        elbow_angle = Constants.Arm.ElbowSetpoints.STOWED.angle;
        break;
      case (1):
        shoulder_angle = Constants.Arm.ShoulderSetpoints.DS.angle;
        elbow_angle = Constants.Arm.ElbowSetpoints.DS.angle;
        break;
      default:
        shoulder_angle = Constants.Arm.ShoulderSetpoints.STOWED.angle;
        elbow_angle = Constants.Arm.ElbowSetpoints.STOWED.angle;
        break;
    }

    setShoulderAngle(shoulder_angle);
    setElbowAngle(elbow_angle);
  }

  @Override
  public void stop() {
    rightElbowMotor.stopMotor();
    rightShoulderMotor.stopMotor();
    leftElbowMotor.stopMotor();
    leftShoulderMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left elbow rel", leftElbowRelativeEncoder.getPosition());
    SmartDashboard.putNumber("right elbow rel", rightElbowRelativeEncoder.getPosition());
    SmartDashboard.putNumber("elbow abs", elbowAbsoluteEncoder.getDistance());

    SmartDashboard.putNumber("left shoulder rel", leftShoulderRelativeEncoder.getPosition());
    SmartDashboard.putNumber("right soulder rel", rightShoulderRelativeEncoder.getPosition());
    SmartDashboard.putNumber("shoulder abs encoder", shoulderAbsoluteEncoder.get());
    SmartDashboard.putNumber("shoulder abs dist", shoulderAbsoluteEncoder.getDistance());

    elbowPIDController.setP(Constants.Arm.elbowKP.get());
    shoulderPIDController.setP(Constants.Arm.shoulderKP.get());
  }

}
