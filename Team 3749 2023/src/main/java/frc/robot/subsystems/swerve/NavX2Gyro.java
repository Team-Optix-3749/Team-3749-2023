package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;


public class NavX2Gyro implements GyroIO{

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);


    public NavX2Gyro() {
        gyro.calibrate();
        new Thread(() -> {
            try {
                while (gyro.isCalibrating()){
                    
                }
                gyro.reset();

            } catch (Exception e) {
            }
        }).start();
    }

    @Override
    public void updateData(GyroData data){
        data.yaw = gyro.getYaw();
        data.pitch = gyro.getPitch();
        data.roll = gyro.getRoll();
        data.isCalibrating = gyro.isCalibrating();
        data.connected = gyro.isConnected();


    }
    @Override
    public void resetGyro(){
        gyro.reset();

    }
    
}
