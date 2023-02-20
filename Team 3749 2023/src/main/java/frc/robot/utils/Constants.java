package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/***
 * Stores constant variables within subclasses for different subsystems. Such
 * constant values can include motor IDs, motor speed, PID constants, etc...
 */
public final class Constants {
    public static final RobotMode ROBOT_MODE = RobotMode.REAL;

    public static final class Claw {
        public static final int claw_id = 22;

        public static final SmartData<Double> speed = new SmartData<Double>("Claw Speed", 0.5);

        // current of above 60 Amps will produce high temperatures
        public static final SmartData<Integer> currentLimit = new SmartData<Integer>("Claw current limit", 45);

        // PID values
        public static final SmartData<Double> kP = new SmartData<Double>("Claw kP", .05);
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

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

        // mass of forearm is wrong - should be 1 pound
        public static final double forearm_mass = 11.3398; // forearm + claw mass (20 + 5 lbs)

        public static final int number_of_motors = 2;

        public static SmartData<Double> elbowKP = new SmartData<Double>("Elbow kP", 0.015);
        public static SmartData<Double> elbowSimKP = new SmartData<Double>("Elbow Sim kP", 0.5);

        public static SmartData<Double> shoulderKP = new SmartData<Double>("Shoulder kP", 0.015);
        public static SmartData<Double> shoulderSimKP = new SmartData<Double>("Shoulder Sim kP", 2.0);

        public static final double sim_encoder_dist_per_pulse = 2.0 * Math.PI / 4096;

        // encoder values (0.0 - 1.0)
        public static final double shoulder_offset = .08;
        public static final double elbow_offset = .272;

        public static final double shoulder_min_angle = 130;
        public static final double shoulder_max_angle = 220;

        public static final double elbow_min_angle = -75;
        public static final double elbow_max_angle = 260;

        public static enum ShoulderSetpoints {
            ZERO(0),
            STOWED(190),
            STING(220),
            DOUBLE_SUBSTATION(200),
            TOP_INTAKE(150),
            CONE_TOP(140),
            CONE_MID(185),
            CUBE_TOP(150),
            CUBE_MID(180);

            public final double angle;

            ShoulderSetpoints(double angle) {
                this.angle = angle;
            }

        }

        public static enum ElbowSetpoints {
            ZERO(0),
            STOWED(25),
            STING(50),
            DOUBLE_SUBSTATION(80),
            TOP_INTAKE(53),
            CONE_TOP(160),
            CONE_MID(90),
            CUBE_TOP(140),
            CUBE_MID(80);

            public final double angle;

            ElbowSetpoints(double angle) {
                this.angle = angle;
            }
        }

        public static enum ArmSetpoints {
            ZERO(ShoulderSetpoints.ZERO.angle, ElbowSetpoints.ZERO.angle),
            STOWED(ShoulderSetpoints.STOWED.angle, ElbowSetpoints.STOWED.angle),
            STING(ShoulderSetpoints.STING.angle, ElbowSetpoints.STING.angle),
            DOUBLE_SUBSTATION(ShoulderSetpoints.DOUBLE_SUBSTATION.angle, ElbowSetpoints.DOUBLE_SUBSTATION.angle),
            TOP_INTAKE(ShoulderSetpoints.TOP_INTAKE.angle, ElbowSetpoints.TOP_INTAKE.angle),
            CONE_TOP(ShoulderSetpoints.CONE_TOP.angle, ElbowSetpoints.CONE_TOP.angle),
            CONE_MID(ShoulderSetpoints.CONE_MID.angle, ElbowSetpoints.CONE_MID.angle),
            CUBE_TOP(ShoulderSetpoints.CUBE_TOP.angle, ElbowSetpoints.CUBE_TOP.angle),
            CUBE_MID(ShoulderSetpoints.CUBE_MID.angle, ElbowSetpoints.CUBE_MID.angle);

            public final double[] angles;

            ArmSetpoints(double shoulder_angle, double elbow_angle) {
                this.angles = new double[]{shoulder_angle, elbow_angle};
            }
        }
    }

    public static enum RobotMode {
        REAL,
        SIMULATION
    }
}