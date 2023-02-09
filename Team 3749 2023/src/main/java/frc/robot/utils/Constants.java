package frc.robot.utils;

import edu.wpi.first.wpilibj.util.Color;

/***
 * Stores constant variables within subclasses for different subsystems. Such
 * constant values can include motor IDs, motor speed, PID constants, etc...
 */
public final class Constants {

    public static final class Base {
        public static final int neo_id = 3749;
        public static final int falcon_id = 6328;
        public static final SmartData<Double> speed = new SmartData<Double>("Base Speed", 2.54);

    }

    public static final class Claw {
        // all values set to zero because we don't know them yet
        public static final int right_side = 0; // right side of the claw (the motor)
        public static final int left_side = 1; // left side of the claw (the motor)

        public static final int speed = 0; // number for the speed
        public static final int stop = 0; // speed 0 = motors stop

        //PID values
        public static final double claw_kP = 0.1;
        public static final double claw_kI = 0;
        public static final double claw_kD = 0;

        public static final Double setpoint_velocity = 0.0;

        //ColorSensor
        public static String Object = null;
    }

    // find ultrasonic docs:
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/ultrasonics-software.html
    // public static final int Ultrasonic_Ping_Port = 0; // ping measures the round-trip echo time of ultrasonic sound to
    //                                                   // determine how far away an object is
    // public static final int Ultrasonic_Echo_Port = 1; // echo sends out ultrasonic signal

    // colors for the color sensor
    public static final Color cube_color = new Color(0.349, 0.207, 0.658);
    public static final Color cone_color = new Color(0.937, 0.643, 0.015);
}
