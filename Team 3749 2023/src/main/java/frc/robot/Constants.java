package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.SmartData;

public class Constants {
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
        public static final double kAutoDriveMaxAccelerationUnitsPerSecond = kTeleDriveMaxAccelerationUnitsPerSecond / 2;
        public static final double kAutoDriveMaxAngularAccelerationUnitsPerSecond = kTeleDriveMaxAngularAccelerationUnitsPerSecond / 2;
    }

    public static final class OIConstants {
        public static final int kPilotControllerPort = 0;
        public static final int kOperatorControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }
    
    public static final class AutoBalancing{
        public static final double max_yaw_offset = 2.2;
        public static final double max_pitch_offset = 2;
        public static final double max_pitch_margin = 3;
        public static final double max_movement_offset = 0.025; // around 1 inch
        public static final double base_speed_mps = 1;
        // FOR TOBY
        public static final double adjust_speed_mps = 0.1;
        public static final double adjusting_distance = 0.05; // In meters, so this is 5 cm
        // Hella Simple
        public static final double adjust_time_length = 0.2; // in seconds
    }

    public static final class AutoConstants{
        public static final Map<String, Command> eventMap = new HashMap<>();
    }
    
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

    public static class VisionConstants {
        public static final int apriltag_pipeline_index = 0;
        public static final int reflective_tape_pipeline_index = 1;
      }
}
