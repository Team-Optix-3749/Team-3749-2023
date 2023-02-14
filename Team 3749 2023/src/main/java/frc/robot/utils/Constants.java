package frc.robot.utils;

/***
 * Stores constant variables within subclasses for different subsystems. Such
 * constant values can include motor IDs, motor speed, PID constants, etc...
 */
public final class Constants {
    public static final class Claw {
        public static final int claw_id = 21;

        public static final SmartData<Double> speed = new SmartData<Double>("Claw Speed", 0.2); 

        // current of above 60 Amps will produce high temperatures
        public static final SmartData<Integer> currentLimit = new SmartData<Integer>("Claw current limit", 45); 

        //PID values
        public static final SmartData<Double> kP = new SmartData<Double>("Claw kP", .05); 
        public static final double kI = 0.0;
        public static final double kD = 0.0;

    }

}
