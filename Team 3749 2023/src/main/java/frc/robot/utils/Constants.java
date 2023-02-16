package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Stores constant variables within subclasses for different subsystems.
 * Such constant values can include motor IDs, motor speed, PID
 * constants, etc...
 */
public final class Constants {

    public static final RobotMode ROBOT_MODE = RobotMode.REAL;

    public static final class Arm {

        public static final SendableChooser<Integer> controlMode = new SendableChooser<Integer>();

        public static final int left_shoulder_id = 15;
        public static final int right_shoulder_id = 16;
        public static final int left_elbow_id = 17;
        public static final int right_elbow_id = 18;

        public static final double shoulder_reduction = 250;
        public static final double elbow_reduction = 200;

        // inches
        public static final double bicep_length = 25; // hypotenuse
        public static final double forearm_length = 30;
        public static final double claw_length = 13;

        // kilograms
        public static final double bicep_mass = 2.26796; // 5 lbs
        public static final double forearm_mass = 11.3398; // forearm + claw mass (20 + 5 lbs)

        public static final int number_of_motors = 2;

        public static SmartData<Double> elbowKP = new SmartData<Double>("Elbow kP", 0.02);
        public static SmartData<Double> elbowSimKP = new SmartData<Double>("Elbow Sim kP", 0.5);

        public static SmartData<Double> shoulderKP = new SmartData<Double>("Shoulder kP", 0.02);
        public static SmartData<Double> shoulderSimKP = new SmartData<Double>("Shoulder Sim kP", 2.0);

        public static final double sim_encoder_dist_per_pulse = 2.0 * Math.PI / 4096;

        // encoder values (0.0 - 1.0)
        // public static final double shoulder_offset = .128;
        public static final double shoulder_offset = .08;
        public static final double elbow_offset = .272;

        public static final double shoulder_min_angle = 90;
        public static final double shoulder_max_angle = 210;

        public static final double elbow_min_angle = -75;
        public static final double elbow_max_angle = 260;

        public static enum ShoulderSetpoints {
            ZERO(0),
            STOWED(190),
            DS(200);

            public final double angle;

            ShoulderSetpoints(double angle) {
                this.angle = angle;
            }

        }

        public static enum ElbowSetpoints {
            ZERO(0),
            STOWED(25),
            DS(73);

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
