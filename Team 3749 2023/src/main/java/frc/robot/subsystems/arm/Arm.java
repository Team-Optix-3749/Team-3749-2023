package frc.robot.subsystems.arm;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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


    // Shoulder motor
    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.Arm.right_shoulder_id, MotorType.kBrushless);
    private final DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(9);
    private final PIDController shoulderPIDController = new PIDController(Constants.Arm.shoulder_kP, 0, 0);

    // Elbow motor
    private final CANSparkMax elbowMotor = new CANSparkMax(Constants.Arm.left_elbow_id, MotorType.kBrushless);
    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(8);
    private final PIDController elbowPIDController = new PIDController(Constants.Arm.elbow_kP, 0, 0);

    // safety stow
    private Translation2d position = new Translation2d(0.3, -0.2);
    private ArmSetpoints currentSetpoint = ArmSetpoints.STOW;

    private ShuffleData<Double> armCacheX = new ShuffleData<Double>("Arm", "Arm Cache X", 0.0);
    private ShuffleData<Double> armCacheY = new ShuffleData<Double>("Arm", "Arm Cache Y", 0.0);
    private ShuffleData<Double> armX = new ShuffleData<Double>("Arm", "Arm X", 0.0);
    private ShuffleData<Double> armY = new ShuffleData<Double>("Arm", "Arm Y", 0.0);
    private ShuffleData<Double> shoulderVoltage = new ShuffleData<Double>("Arm", "Shoulder Voltage", 0.0);
    private ShuffleData<Double> shoulderFF = new ShuffleData<Double>("Arm", "Shoulder FF Output", 0.0);
    private ShuffleData<Double> shoulderPID = new ShuffleData<Double>("Arm", "Shoulder PID Output", 0.0);
    private ShuffleData<Double> shoulderAngle = new ShuffleData<Double>("Arm", "Shoulder Angle", 0.0);
    private ShuffleData<Double> elbowVoltage = new ShuffleData<Double>("Arm", "Elbow Voltage", 0.0);
    private ShuffleData<Double> elbowFF = new ShuffleData<Double>("Arm", "Elbow PID Output", 0.0);
    private ShuffleData<Double> elbowPID = new ShuffleData<Double>("Arm", "Elbow FF Output", 0.0);
    private ShuffleData<Double> elbowAngle = new ShuffleData<Double>("Arm", "Elbow Angle", 0.0);

    private boolean kill = false;

    public Arm() {
        shoulderMotor.restoreFactoryDefaults();
        elbowMotor.restoreFactoryDefaults();
        
        // elbow offset is done in the get angle method
        shoulderAbsoluteEncoder.setPositionOffset(Constants.Arm.shoulder_offset);
        shoulderAbsoluteEncoder.setDistancePerRotation(360);
        elbowAbsoluteEncoder.setDistancePerRotation(360);

        elbowMotor.setInverted(true);
        shoulderMotor.setInverted(false);

        shoulderPIDController.setTolerance(0);
        elbowPIDController.setTolerance(0);

        shoulderMotor.setSmartCurrentLimit(35, 60);
        elbowMotor.setSmartCurrentLimit(35, 60);

        setIdleMode(IdleMode.kCoast);
    }

    /**
     * Set shoulder motor voltage. The voltage will not be set if kill is true
     * 
     * @param voltage
     */
    public void setShoulderVoltage(double voltage) {
        if (kill) {
            shoulderMotor.setVoltage(0);
        } else {
            shoulderMotor.setVoltage(voltage);
        }
    }
    
    /**
     * Set elbow motor voltage. The voltage will not be set if kill is true
     * 
     * @param voltage
     */
    public void setElbowVoltage(double voltage) {
        if (kill) {
            elbowMotor.setVoltage(0);
        } else {
            elbowMotor.setVoltage(voltage);
        }
    }

    /**
     * Stop both arm motors
     */
    public void stop() {
        elbowMotor.stopMotor();
        shoulderMotor.stopMotor();
    }

    /**
     * Stop shoulder motor
     */
    public void stopShoulder() {
        shoulderMotor.stopMotor();
    }

    /**
     * Stop elbow motor
     */
    public void stopElbow() {
        elbowMotor.stopMotor();
    }

    /**
     * Get current shoulder angle
     * 
     * @return shoulder angle as double
     */
    public double getShoulderAngle() {
        return (shoulderAbsoluteEncoder.getAbsolutePosition()-Constants.Arm.shoulder_offset) * 360 ;
        // return (shoulderAbsoluteEncoder.getAbsolutePosition() * 360.0 );
    }

    /**
     * Get current elbow angle
     * 
     * @return elbow angle as double
     */
    public double getElbowAngle() {
        return new Rotation2d(Math.toRadians(elbowAbsoluteEncoder.getAbsolutePosition() * 360 - 180))
                .rotateBy(new Rotation2d(Math.toRadians(180))).getDegrees() - Constants.Arm.elbow_offset;
        // return new Rotation2d(Math.toRadians(elbowAbsoluteEncoder.getAbsolutePosition() * 360 - 180))
        //     .rotateBy(new Rotation2d(Math.toRadians(180))).getDegrees();
    }

    /**
     * Set arm to Translation2d position
     * 
     * @param pos
     */
    public void setArmPosition(Translation2d pos) {
        position = pos;
    }


    /**
     * Move arm to set position using PID and DJ FF
     * 
     * @throws Exception
     */
    
    private void moveArm() throws Exception {
        double shoulderAngle = ArmKinematics.inverse(position.getX(), position.getY()).getFirst();
        double elbowAngle = ArmKinematics.inverse(position.getX(), position.getY()).getSecond();

        double[] feedForwardOutput = dynamics.feedforward(VecBuilder.fill(shoulderAngle, elbowAngle)).getData();

        setShoulderVoltage(shoulderPIDController.calculate(getShoulderAngle(), shoulderAngle) + feedForwardOutput[0]);
        setElbowVoltage(elbowPIDController.calculate(getElbowAngle(), elbowAngle) + feedForwardOutput[1]);

        shoulderPID.set(shoulderPIDController.calculate(getShoulderAngle(), shoulderAngle));
        shoulderFF.set(feedForwardOutput[0]);

        elbowPID.set(elbowPIDController.calculate(getElbowAngle(), elbowAngle));
        elbowFF.set(feedForwardOutput[1]);
    }


    /**
     * Get current arm pose as Translation2d
     * 
     * @return arm coordianates as Translation2d
     */

    public Translation2d getArmCoordinate() {
        return ArmKinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle()));
    }

    public void setCurrentSetpoint(ArmSetpoints setpoint){
        currentSetpoint = setpoint;
    }

    public ArmSetpoints getCurrentSetpoint(){
        return currentSetpoint;
    }

    /**
     * Toggles the kill boolean and sets teh idle mode to brake if killed, and back
     * to coast if unkilled
     */
    public void toggleKillArm() {
        kill = !kill;

        if (kill) {
            setIdleMode(IdleMode.kBrake);
            System.out.println("KILLED ARM");
        } else {
            setIdleMode(IdleMode.kCoast);
            System.out.println("UNKILLED ARM");
        }
    }

    /**
     * Use this method to test & log the FF output for a given XY
     * 
     * @param x
     * @param y
     * @throws Exception Terminates the process in the event the XY exceeds the arm radius
     */
    public void ffTesting(double x, double y) throws Exception {
        double shoulderAngle = ArmKinematics.inverse(x, y).getFirst();
        double elbowAngle = ArmKinematics.inverse(x, y).getSecond();

        double[] feedForwardOutput = dynamics.feedforward(VecBuilder.fill(shoulderAngle, elbowAngle)).getData();

        shoulderFF.set(feedForwardOutput[0]);
        elbowFF.set(feedForwardOutput[1]);
    }

    /**
     * @param mode IdleMode of the arm
     */
    public void setIdleMode(IdleMode mode){
        shoulderMotor.setIdleMode(mode);
        elbowMotor.setIdleMode(mode);
    }

    @Override
    public void periodic() {
        try {
            moveArm();
        } catch (Exception e) {
            System.out.println(e);
        }
        
        shoulderAngle.set(getShoulderAngle());
        elbowAngle.set(getElbowAngle());

        armCacheX.set(position.getX());
        armCacheY.set(position.getY());

        var kinematicsOutput = ArmKinematics.forward(Math.toRadians(getShoulderAngle()), Math.toRadians(getElbowAngle()));

        armX.set(kinematicsOutput.getX());
        armY.set(kinematicsOutput.getY());

        shoulderVoltage.set(shoulderMotor.getBusVoltage() * shoulderMotor.getAppliedOutput());
        elbowVoltage.set(elbowMotor.getBusVoltage() * elbowMotor.getAppliedOutput());
    }
}