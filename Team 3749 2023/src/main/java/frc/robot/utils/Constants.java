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

    public static final class Arm {
        public static final int neo_motor_telescope_port = 13; // Change this value later
        public static final int neo_motor_elevator_port = 15; // Change this value later

        public static final int neo_motor_lower_port = 0; // Change this value later
        public static final int neo_motor_upper_port = 1; // Change this value later

        public static final int neo_motor_telescope_speed = 1; // Change this value later
        public static final int neo_motor_elevator_speed = 1; // Change this value later

        public static final int neo_motor_lower_speed = 1; // Change this value later
        public static final int neo_motor_upper_speed = 1; // Change this value later

        public static final int neo_motor_lower_stop = 0; // Change this value later
        public static final int neo_motor_upper_stop = 0; // Change this value later

        public static final int number_of_motors = 2;

        public static final int kp = 40;
        public static final int ki = 0;
        public static final int kd = 0;

        public static final int max_velocity = 2;
        public static final int max_acceleration = 5;
    }

    public static final class Simulation { // these values are going to need to change (along with simulation)
        // distance per pulse = (angle per revolution) / (pulses per revolution)
        //  = (2 * PI rads) / (4096 pulses)
        public static final double arm_encoder_dist_per_pulse = 2.0 * Math.PI / 4096;
        
        // Simulation classes help us simulate what's going on, including gravity.
        public static final double arm_reduction = 600;
        public static final double arm_top_mass = 10.0; // Kilograms
        public static final double arm_top_length = Units.inchesToMeters(38.5);
        public static final double arm_bottom_mass = 4.0; // Kilograms
        public static final double arm_bottom_length = Units.inchesToMeters(27);

        public static final int arm_top_min_angle = -75; 
        public static final int arm_top_max_angle = 260; 
        public static final int arm_bottom_min_angle = 30; 
        public static final int arm_bottom_max_angle = 150; 

        //SETPOINTS FOR PRESETS MODE (Uses Virtual 4 Bar Mode for smooth movement)
        public static final int stowed_bottom = 90;
        public static final int stowed_top = 260;

        public static final int intake_bottom = 135;
        public static final int intake_top = 265;

        public static final int double_substation_bottom = 60;
        public static final int double_substation_top = 185;

        public static final int score_floor_bottom = 120;
        public static final int score_floor_top = 255;

        public static final int score_mid_bottom = 95;
        public static final int score_mid_top = 195;

        public static final int score_high_bottom = 135;
        public static final int score_high_top = 160;
    }

    public static final class Controller {
        public static final int joystick_port = 0;
    }
}
