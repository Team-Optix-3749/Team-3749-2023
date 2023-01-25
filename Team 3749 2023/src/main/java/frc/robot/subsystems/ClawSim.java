package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ClawSim extends SubsystemBase {

    private CANSparkMax neo = new CANSparkMax(Constants.Base.neo_id, MotorType.kBrushless);
    private WPI_TalonFX falcon = new WPI_TalonFX(Constants.Base.falcon_id);

    private MotorControllerGroup base = new MotorControllerGroup(neo, falcon);

    // Initializes the clawSim subsystem
    public ClawSim() {
        neo.setInverted(true);  
        falcon.setNeutralMode(NeutralMode.Brake);

        //Constants.Base.speed.set(new Double(16.90));
    }

    /***
     * Sets the speed. Value is between -1.0 and 1.0
     * 
     * @param percent_speed
     */
    public void set(double percent_speed) {
        base.set(percent_speed);
    }

    /***
     * Gets the speed. Value is between -1.0 and 1.0
     * 
     * @return speed of the first motor in the motor controller group (neo)
     */
    public double get() {
        return base.get();
    }

    // Runs every 20 ms
    @Override
    public void periodic() {

    }

}