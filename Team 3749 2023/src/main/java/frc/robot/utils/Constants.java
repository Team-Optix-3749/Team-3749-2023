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

    public static final class SwerveModuleOld {
        public static final double wheel_diameter = Units.inchesToMeters(4); // in meters
        // make sure that gear ratios are updated
        public static final double drive_motor_gear_ratio = 1 / 42;
        public static final double turning_motor_gear_ratio = 1 / 42;
        public static final double drive_encoder_rotations_to_meter = drive_motor_gear_ratio * Math.PI * wheel_diameter;
        public static final double turning_encoder_rotations_to_meter = turning_motor_gear_ratio * 2 * Math.PI;
        public static final double drive_encoder_RPM_to_MPS = drive_encoder_rotations_to_meter / 60; // in meters per
                                                                                                     // second
        public static final double turning_encoder_RPM_to_MPS = turning_encoder_rotations_to_meter / 60; // in meters
                                                                                                         // per second

        public static final SmartData<Double> turning_p = new SmartData<Double>("Turning-P", 0.5); 
    }

    public static final class SwerveModuleNew {
        public static final double wheel_radius = Units.inchesToMeters(2);
        public static final double encoder_resolution = 4096; // From rev data sheet

        public static final double max_angular_velocity = DrivetrainNew.max_speed;
        public static final double max_angular_acceleration = 2 * Math.PI; // radians per second squared

        // meters
        public static final double drive_encoder_rotations_to_meter = 2 * Math.PI
                * Constants.SwerveModuleNew.wheel_radius / Constants.SwerveModuleNew.encoder_resolution;
        public static final double turning_encoder_rotations_to_meter = 2 * Math.PI
                / Constants.SwerveModuleNew.encoder_resolution;

        // meters per second
        public static final double drive_encoder_RPM_to_MPS = drive_encoder_rotations_to_meter / 60;
        public static final double turning_encoder_RPM_to_MPS = turning_encoder_rotations_to_meter / 60;
    }

    public static final class DrivetrainNew {
        public static final double max_speed = 3.0; // 3 meters per second
        public static final double max_angular_speed = Math.PI; // 1/2 rotation per second

        public static final boolean gyro_reversed = false;

        // Distance between two wheels on opposite sides
        public static final double track_width = Units.inchesToMeters(18);
        // Distance between right and left wheels
        public static final double wheel_base = Units.inchesToMeters(18);
        // Distance between front and back wheels

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(wheel_base / 2, -track_width / 2),
                new Translation2d(wheel_base / 2, track_width / 2),
                new Translation2d(-wheel_base / 2, -track_width / 2),
                new Translation2d(-wheel_base / 2, track_width / 2));
        
        public static final SmartData<Double> driveKV = new SmartData<Double>("Drive kV", 3.0);
        public static final SmartData<Double> turningKV = new SmartData<Double>("Turning kV", 0.5);
    }

    public static final class DrivetrainOld {
        // Distance between two wheels on opposite sides
        public static final double track_width = Units.inchesToMeters(18);
        // Distance between right and left wheels
        public static final double wheel_base = Units.inchesToMeters(18);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheel_base / 2, -track_width / 2),
                new Translation2d(wheel_base / 2, track_width / 2),
                new Translation2d(-wheel_base / 2, -track_width / 2),
                new Translation2d(-wheel_base / 2, track_width / 2));

        // Front _left_ module
        public static final int front_left_drive_id = 1;
        public static final int front_left_turning_id = 2;
        public static final boolean front_left_turning_encoder_reversed = true;
        public static final boolean front_left_drive_encoder_reversed = true;
        public static final int front_left_drive_absolute_encoder_port = 0;
        public static final boolean front_left_drive_absolute_encoder_reversed = false;
        public static final double front_left_drive_absolute_encoder_offset_rad = 0.0;

        // Back _left_ module
        public static final int back_left_drive_id = 5;
        public static final int back_left_turning_id = 6;
        public static final boolean back_left_turning_encoder_reversed = true;
        public static final boolean back_left_drive_encoder_reversed = true;
        public static final int back_left_drive_absolute_encoder_port = 2;
        public static final boolean back_left_drive_absolute_encoder_reversed = false;
        public static final double back_left_drive_absolute_encoder_offset_rad = 0;

        // Front right module
        public static final int front_right_drive_id = 3;
        public static final int front_right_turning_id = 4;
        public static final boolean front_right_turning_encoder_reversed = true;
        public static final boolean front_right_drive_absolute_encoder_reversed = false;
        public static final int front_right_drive_absolute_encoder_port = 1;
        public static final boolean front_right_drive_encoder_reversed = false;
        public static final double front_right_drive_absolute_encoder_offset_rad = 0;

        // Back right module
        public static final int back_right_drive_id = 7;
        public static final int back_right_turning_id = 8;
        public static final boolean back_right_turning_encoder_reversed = true;
        public static final boolean back_right_drive_encoder_reversed = false;
        public static final int back_right_drive_absolute_encoder_port = 3;
        public static final boolean back_right_drive_absolute_encoder_reversed = false;
        public static final double back_right_drive_absolute_encoder_offset_rad = 0;

        public static final double physical_max_speed_meters_per_second = 5;
        public static final double physical_max_angular_speed_radians_per_second = 2 * 2 * Math.PI;

        public static final double tele_drive_max_speed_meters_per_second = physical_max_speed_meters_per_second / 4;
        public static final double tele_drive_max_angular_speed_radians_per_second = physical_max_speed_meters_per_second
                / 4;
        public static final double tele_drive_max_acceleration_units_per_second = 3;
        public static final double tele_drive_max_angular_acceleration_units_per_second = 3;

        public static final double deadband = 0.05;

        public static final double min_balance_angle = 1.75;
    }

    public static enum SwerveENUMS {
        FRONT_RIGHT,
        FRONT_LEFT,
        BACK_RIGHT,
        BACK_LEFT;
    }

}
