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

    // all values set to zero because we don't know them yet
    public static final int right_side = 0; // right side of the claw (the motor)
    public static final int left_side = 1; // left side of the claw (the motor)

    public static final int speed = 0; // number for the speed

    // find ultrasonic docs: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/ultrasonics-software.html
    public static final int UltrasonicPingPort = 0; // ping measures the round-trip echo time of ultrasonic sound to determine how far away an object is
    public static final int UltrasonicEchoPort = 1; // echo sends out ultrasonic signal

}
