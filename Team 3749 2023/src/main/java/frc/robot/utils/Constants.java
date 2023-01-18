package frc.robot.utils;

import edu.wpi.first.math.util.Units;

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
    
    public static final class Controller {
        public static final int joystick_port = 0;
    }

    public static final class Arm {
        public static final int neo_motor_lower_id_1 = 0; // Change this value later
        public static final int neo_motor_lower_id_2 = 1; //Change this value later
        public static final int neo_motor_upper_id_1 = 2;
        public static final int neo_motor_upper_id_2 = 3; // Change this value later

        //public static final int neo_motor_lower_left_speed = 1; // Change this value later
        //public static final int neo_motor_lower_right_speed = 1; //Change this val later

        // These speeds apply for both lower motors and both upper motors respectively
        public static final int neo_motor_lower_speed = 1; // Change this value later
        public static final int neo_motor_upper_speed = 1; // Change this value later
        
        public static final int number_of_motors = 2;

        public static final double kp = 0.5;

        // Note: ki must be set to 0 to avoid integral windup, feedforward will be used to account for the error instead
        public static final double ki = 0;
        public static final double kd = 0;

        public static final int max_velocity = 2;
        public static final int max_acceleration = 5;
    }
}
