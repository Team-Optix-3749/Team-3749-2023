package frc.robot.utils;

/***
 * Stores constant variables within subclasses for different subsystems. Such
 * constant values can include motor IDs, motor speed, PID constants, etc...
 */
public final class Constants {
    public static final class Claw {
        public static final int right_side = 21; // right side of the claw (the motor)
        public static final int left_side = 22; // left side of the claw (the motor)

        public static final SmartData<Double> speed = new SmartData<Double>("Claw Speed", 0.0); // number for the speed
        public static final int stop = 0; // speed 0 = motors stop

        //PID values
        public static final double claw_kP = 0.1;
        public static final double claw_kI = .1;
        public static final double claw_kD = .1;

        public static final Double setpoint_velocity = 0.5;
    }

}
