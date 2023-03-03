package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;

public class Constants {
    /***
     * 
     * @param margin how close the values need to be to return true. Use a positive
     *               number
     * @param a      the first number
     * @param b      the second number
     * @return true if it is within the margin, false if not
     */
    public static boolean withinMargin(double margin, double a, double b) {
        if (a + margin >= b && a - margin <= b) {
            return true;
        }
        return false;
    }

    /***
     * 
     * @param margin how close the values need to be to return true. Use a positive
     *               number
     * @param a      the first translation
     * @param b      the second translation
     * @return true if it is within the margin, false if not
     */
    public static boolean withinMargin(double margin, Translation2d a, Translation2d b) {
        // if X is within margin
        if (a.getX() + margin >= b.getX() && a.getX() - margin <= b.getX()) {
            // if Y is within margin
            if (a.getY() + margin >= b.getY() && a.getY() - margin <= b.getY()) {

                return true;
            }
        }
        return false;
    }

    public static final class Claw {
        public static final int claw_id = 22;
        public static final double idleVoltage = 1;
        public static final double releaseObjectVoltage = -3;
        public static final double intakeVoltage = 6;
    }

    public static final class Arm {
        // public static final double elbow_kP = 0.3;
        public static final double elbow_kP = 0.2;
        // public static final double shoulder_kP = 0.2;
        public static final double shoulder_kP = 0.15;

        public static final double elbow_length = 1.016;
        public static final double elbow_cg_radius = 0.71;
        public static final double elbow_mass = 4.4;
        public static final double elbow_moi = SingleJointedArmSim.estimateMOI(elbow_length, elbow_mass);

        public static final double shoulder_length = 0.7239;
        public static final double shoulder_cg_radius = 0.245;
        public static final double shoulder_mass = 0.5;
        public static final double shoulder_moi = SingleJointedArmSim.estimateMOI(shoulder_length, shoulder_mass);

        public static final int left_shoulder_id = 15;
        public static final int right_shoulder_id = 16;
        public static final int left_elbow_id = 17;
        public static final int right_elbow_id = 18;

        // (angle without offset - desired angle) / 360
        public static final double shoulder_offset = (31.0) / 360.0;

        // just the angle offset in degrees
        public static final double elbow_offset = 38.0;

        public static final double shoulder_min_angle = 30;
        public static final double shoulder_max_angle = 140;

        public static final double elbow_min_angle = -75;
        public static final double elbow_max_angle = 260;

        public static final double maxSpeedMPS = 7;
        public static final double maxAccelerationMPS = 10;
        
        public static enum ArmSetpoints {
            ZERO,
            STOWED,
            STING,
            DOUBLE_SUBSTATION,
            TOP_INTAKE,
            CONE_TOP,
            CONE_MID,
            CUBE_TOP,
            CUBE_MID;

        }
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
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(17.5);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(17.5);
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
        public static final double kDeadband = 0.1;
    }

    public static final class AutoBalancing {
        public static final double max_yaw_offset = 2.2;
        public static final double max_pitch_offset = 2;
        public static final double max_pitch_margin = 3;
        public static final double max_movement_offset = 0.025; // around 1 inch

        public static final double base_speed_mps = 0.75;

        public static final double adjust_speed_mps = 0.1;
        public static final double adjusting_distance = 0.05; // In meters, so this is 5 cm
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