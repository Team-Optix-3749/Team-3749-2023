package frc.robot.utils;

/***
 * Stores constant variables within subclasses for different subsystems.
 * Such constant values can include motor IDs, motor speed, PID
 * constants, etc...
 */
public final class Constants {

    public static final RobotMode robotState = RobotMode.REAL;
    public static final class Base {
        public static final int neo_id = 3749;
        public static final int falcon_id = 6328;
        public static final SmartData<Double> speed = new SmartData<Double>("Base Speed", 2.54);
    }

    public static final class Lights {
        public static final int led_length = 20; // change this value
        public static final int led_port = 0; // change this value
        public static final int[] light1 = { 0, 255, 0 };
        public static final int[] light2 = { 255, 255, 255 };
    }

    public static final class Controller {
        public static final int joystick_port = 0;
    }

    public static final class Arm {
        public static final int left_shoulder_id = 15;
        public static final int right_shoulder_id = 16;
        public static final int left_elbow_id = 17;
        public static final int right_elbow_id = 18;

        public static final double shoulder_reduction = 200;
        public static final double elbow_reduction = 250;
        
        // inches
        public static final double bicep_length = 25; // hypotenuse
        public static final double forearm_length = 26;
        public static final double claw_length = 13;
        
        // kilograms
        public static final double bicep_mass = 2; // 5 lbs
        public static final double forearm_mass = 2; // forearm + claw mass (20 + 5 lbs)

        public static final int number_of_motors = 2;

        public static final double kp = 0.5;

        public static SmartData<Double> elbowSetpoint = new SmartData<Double>("Elbow Setpoint", -90.0);
        public static SmartData<Double> shoulderSetpoint = new SmartData<Double>("Shoulder Setpoint", -90.0);

        // Note: ki must be set to 0 to avoid integral windup, feedforward will be used
        // to account for the error instead
        public static final double ki = 0;
        public static final double kd = 0;


        public static final double shoulder_min_angle = 30;
        public static final double shoulder_max_angle = 150;

        public static final double elbow_min_angle = -75;
        public static final double elbow_max_angle = 260;

        public static enum ShoulderSetpoints {
            ZERO(0),
            STOWED(0),
            GROUND_INTAKE(132.0);

            public final double angle;

            ShoulderSetpoints(double angle) {
                this.angle = angle;
            }

        }

        public static enum ElbowSetpoints {
            ZERO(0),
            STOWED(0),
            GROUND_INTAKE(111.0);

            public final double angle;

            ElbowSetpoints(double angle) {
                this.angle = angle;
            }

        }

    }

    public static enum RobotMode {
        REAL,
        SIMULATION
    }
}
