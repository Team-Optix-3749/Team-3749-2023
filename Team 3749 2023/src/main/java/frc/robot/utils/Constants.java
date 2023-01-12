package frc.robot.utils;

/***
 * Stores constant variables within subclasses for different subsystems.
 * Such constant values can include motor IDs, motor speed, PID
 * constants, etc...
 */
public final class Constants {

    public static final class Base {
        public static final int neo_id = 3749;
        public static final int falcon_id = 6328;
        public static final SmartData<Double> speed = new SmartData<Double>("Base Speed", 2.54);
    }
    public static final class Lights {
        public static final int led_length = 20; //change this value
        public static final int led_port = 0; //change this value
        public static final int[] light1 = {0, 255, 0};
        public static final int[] light2 = {255, 255, 255};
    }

    public static final class Arm {
        public static final int neo_motor_telescope_port = 0; // Change this value later
        public static final int neo_motor_elevator_port = 0; // Change this value later

        public static final int neo_motor_telescope_speed = 1; // Change this value later
        public static final int neo_motor_elevator_speed = 1; // Change this value later

        public static final int neo_motor_telescope_stop = 0; // Change this value later
        public static final int neo_motor_elevator_stop = 0; // Change this value later
    }
}
