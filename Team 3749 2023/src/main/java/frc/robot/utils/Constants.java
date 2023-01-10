package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

    public static final class SwerveModule {
        public static final double wheel_diameter = Units.inchesToMeters(4); // in meters
        // make sure that gear ratios are updated
        public static final double drive_motor_gear_ratio = 1 / 5.8462;
        public static final double turning_motor_gear_ratio = 1 / 18.0;
        public static final double drive_encoder_rotations_to_meter = drive_motor_gear_ratio * Math.PI * wheel_diameter;
        public static final double turning_encoder_rotations_to_meter = turning_motor_gear_ratio * 2 * Math.PI;
        public static final double drive_encoder_RPM_to_MPS = drive_encoder_rotations_to_meter / 60; // in meters per second
        public static final double turning_encoder_RPM_to_MPS = turning_encoder_rotations_to_meter / 60; // in meters per second
        public static final double turning_p = 0.5;
    }

    public static final class Drivetrain {

        // Distance between two wheels on opposite sides
        public static final double track_width = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double wheel_base = Units.inchesToMeters(25.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheel_base / 2, -track_width / 2),
                new Translation2d(wheel_base / 2, track_width / 2),
                new Translation2d(-wheel_base / 2, -track_width / 2),
                new Translation2d(-wheel_base / 2, track_width / 2));

        // Front _left_ module
        public static final int front_left_drive_id = 8;
        public static final int front_left_Turning_id = 7;
        public static final boolean front_left_TurningEncoderReversed = true;
        public static final boolean front_left_driveEncoderReversed = true;
        public static final int front_left_driveAbsoluteEncoderPort = 0;
        public static final boolean front_left_driveAbsoluteEncoderReversed = false;
        public static final double front_left_driveAbsoluteEncoderOffsetRad = -0.254;

        // Back _left_ module
        public static final int kBack_left_drive_id = 2;
        public static final int kBack_left_Turning_id = 1;
        public static final boolean kBack_left_TurningEncoderReversed = true;
        public static final boolean kBack_left_driveEncoderReversed = true;
        public static final int kBack_left_driveAbsoluteEncoderPort = 2;
        public static final boolean kBack_left_driveAbsoluteEncoderReversed = false;
        public static final double kBack_left_driveAbsoluteEncoderOffsetRad = -1.252;
        
        // Front right module
        public static final int frontRightdrive_id = 6;
        public static final int frontRightTurning_id = 5;
        public static final boolean frontRightTurningEncoderReversed = true;
        public static final boolean frontRightdriveAbsoluteEncoderReversed = false;
        public static final int frontRightdriveAbsoluteEncoderPort = 1;
        public static final boolean frontRightdriveEncoderReversed = false;
        public static final double frontRightdriveAbsoluteEncoderOffsetRad = -1.816;
        
        // Back right module
        public static final int kBackRightdrive_id = 4;
        public static final int kBackRightTurning_id = 3;
        public static final boolean kBackRightTurningEncoderReversed = true;
        public static final boolean kBackRightdriveEncoderReversed = false;
        public static final int kBackRightdriveAbsoluteEncoderPort = 3;
        public static final boolean kBackRightdriveAbsoluteEncoderReversed = false;
        public static final double kBackRightdriveAbsoluteEncoderOffsetRad = -4.811;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeledriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeledriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeledriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeledriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double deadband = 0.05;
    }

}
