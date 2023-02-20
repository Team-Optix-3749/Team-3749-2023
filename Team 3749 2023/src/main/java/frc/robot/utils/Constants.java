package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Constants {
    public static final RobotMode ROBOT_MODE = RobotMode.REAL;

    public static Arm.ArmSetpoints desired_setpoint = Arm.ArmSetpoints.STOWED;

    public static final class Claw {
        public static final int claw_id = 22;

        public static final SmartData<Double> speed = new SmartData<Double>("Claw Speed", 0.4); 

        // current of above 60 Amps will produce high temperatures
        public static final SmartData<Double> currentLimit = new SmartData<Double>("Claw current limit", 30.0); 

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
        public static final double shoulder_max_angle = 255;

        public static final double elbow_min_angle = -75;
        public static final double elbow_max_angle = 260;

        public static enum ShoulderSetpoints {
            ZERO(0),
            STOWED(190),
            STING(220),
            DOUBLE_SUBSTATION(200),
            TOP_INTAKE(150),
            CONE_TOP(143),
            CONE_MID(188),
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
            CONE_TOP(163),
            CONE_MID(93),
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
                this.angles = new double[] { shoulder_angle, elbow_angle };
            }
        }
    }

    public static enum RobotMode {
        REAL,
        SIMULATION
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1.0 / 6.75;
        public static final double kTurningMotorGearRatio = 1.0 / 12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(17.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(17.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front right
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // back right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); // back left

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 7;
        public static final int kBackRightDriveMotorPort = 5;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackLeftTurningMotorPort = 8;
        public static final int kBackRightTurningMotorPort = 6;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = 0;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = 0 ;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = 0;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = 0;

        public static double kFrontLeftDriveAbsoluteEncoderOffsetDeg = 130.341796875;
        public static double kFrontRightDriveAbsoluteEncoderOffsetDeg = 107.75390625;
        public static double kBackLeftDriveAbsoluteEncoderOffsetDeg = 61.69921875;
        public static double kBackRightDriveAbsoluteEncoderOffsetDeg = 168.75;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

        public static final double kAutoDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kAutoDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kAutoDriveMaxAccelerationUnitsPerSecond = kTeleDriveMaxAccelerationUnitsPerSecond
                / 2;
        public static final double kAutoDriveMaxAngularAccelerationUnitsPerSecond = kTeleDriveMaxAngularAccelerationUnitsPerSecond
                / 2;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

    public static final class AutoBalancing {
        public static final double max_yaw_offset = 2.2;
        public static final double max_pitch_offset = 2;
        public static final double max_pitch_margin = 3;
        public static final double max_movement_offset = 0.025; // around 1 inch
        public static final double base_speed_mps = 0.75;
        // FOR TOBY
        public static final double adjust_speed_mps = 0.1;
        public static final double adjusting_distance = 0.05; // In meters, so this is 5 cm
        // Hella Simple
        public static final double adjust_time_length = 0.2; // in seconds
    }

    public static final class AutoConstants {
        public static final Map<String, Command> eventMap = new HashMap<>();
    }

    public static class VisionConstants {
        public static final int apriltag_pipeline_index = 0;
        public static final int reflective_tape_pipeline_index = 1;
    }
}
