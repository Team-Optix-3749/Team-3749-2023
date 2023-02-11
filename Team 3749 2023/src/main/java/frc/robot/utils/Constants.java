package frc.robot.utils;

/***
 * Stores constant variables within subclasses for different subsystems. Such
 * constant values can include motor IDs, motor speed, PID constants, etc...
 */
public final class Constants {
    public static final class Claw {
        public static final int right_side = 21;
        public static final int left_side = 22; 

        public static final SmartData<Double> speed = new SmartData<Double>("Claw Speed", 0.0); 

        //PID values
        public static final SmartData<Double> kP = new SmartData<Double>("Claw kP", 1.0); 
        public static final double kI = 0.0;
        public static final double kD = 0.0;

    }

}
