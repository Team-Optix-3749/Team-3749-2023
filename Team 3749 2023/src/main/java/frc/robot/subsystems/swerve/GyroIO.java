package frc.robot.subsystems.swerve;


public interface GyroIO {
    public static class GyroData {
        public boolean connected = false;
        public boolean isCalibrating = false;
        public double yaw = 0;
        public double pitch = 0;
        public double roll = 0;
      }
    

    public default void updateData(GyroData data){

    }
    public default void resetGyro(){
        
    }
}
