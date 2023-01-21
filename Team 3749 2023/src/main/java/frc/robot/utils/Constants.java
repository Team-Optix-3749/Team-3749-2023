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

    public static final class SwerveModuleNew {
        public static final double wheel_radius = Units.inchesToMeters(3.5/2);
        public static final double drive_encoder_conversion_factor = 1/12.8;

        public static final double max_angular_velocity = DrivetrainNew.max_speed;
        public static final double max_angular_acceleration = 2 * Math.PI; // radians per second squared



        // meters
        // public static final double drive_encoder_rotations_to_meter = 2 * Math.PI
        //         * wheel_radius * drive_encoder_conversion_factor;
        // public static final double turning_encoder_rotations_to_meter = 2 * Math.PI
        //         / drive_encoder_conversion_factor;

        // // meters per second
        // public static final double drive_encoder_RPM_to_MPS = drive_encoder_rotations_to_meter / 60;
        // public static final double turning_encoder_RPM_to_MPS = turning_encoder_rotations_to_meter / 60;


        // Front _left_ module
        public static final int front_left_drive_id = 1;
        public static final int front_left_turning_id = 2;
        public static final int front_left_absolute_encoder_port = 9;
        public static final boolean front_left_turning_encoder_reversed = true;
        public static final boolean front_left_drive_encoder_reversed = true;
        public static final boolean front_left_drive_absolute_encoder_reversed = false;
        public static final double front_left_drive_absolute_encoder_offset_rad = 0.0;

        // Back _left_ module
        public static final int back_left_drive_id = 7;
        public static final int back_left_turning_id = 8;
        public static final int back_left_absolute_encoder_port = 11;
        public static final boolean back_left_turning_encoder_reversed = true;
        public static final boolean back_left_drive_encoder_reversed = true;
        public static final boolean back_left_drive_absolute_encoder_reversed = false;
        public static final double back_left_drive_absolute_encoder_offset_rad = 0;

        // Front right module
        public static final int front_right_drive_id = 3;
        public static final int front_right_turning_id = 4;
        public static final int front_right_absolute_encoder_port = 10;

        public static final boolean front_right_turning_encoder_reversed = true;
        public static final boolean front_right_drive_absolute_encoder_reversed = false;
        public static final boolean front_right_drive_encoder_reversed = false;
        public static final double front_right_drive_absolute_encoder_offset_rad = 0;

        // Back right module
        public static final int back_right_drive_id = 5;
        public static final int back_right_turning_id = 6;
        public static final int back_right_absolute_encoder_port = 12;
        public static final boolean back_right_turning_encoder_reversed = true;
        public static final boolean back_right_drive_encoder_reversed = false;
        public static final boolean back_right_drive_absolute_encoder_reversed = false;
        public static final double back_right_drive_absolute_encoder_offset_rad = 0;

        public static final double physical_max_speed_meters_per_second = 5;
        public static final double physical_max_angular_speed_radians_per_second = 2 * 2 * Math.PI;


    }

    public static final class DrivetrainNew {
        public static final double max_speed = 3.0; // 3 meters per second
        public static final double max_angular_speed = Math.PI; // 1/2 rotation per second

        public static final boolean gyro_reversed = false;

        // Distance between two wheels on opposite sides
        public static final double track_width = Units.inchesToMeters(17.5);
        // Distance between right and left wheels
        public static final double wheel_base = Units.inchesToMeters(17.5);
        // Distance between front and back wheels

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(wheel_base / 2, -track_width / 2), // front right
                new Translation2d(wheel_base / 2, track_width / 2), // front left
                new Translation2d(-wheel_base / 2, -track_width / 2), // back right
                new Translation2d(-wheel_base / 2, track_width / 2)); // back left
            

                
        // public static final double tele_drive_max_speed_meters_per_second = SwerveModuleNew.physical_max_speed_meters_per_second / 4;
        // public static final double tele_drive_max_angular_speed_radians_per_second = SwerveModuleNew.physical_max_speed_meters_per_second;
        // public static final double tele_drive_max_acceleration_units_per_second = 3;
        // public static final double tele_drive_max_angular_acceleration_units_per_second = 3;

        public static final double deadband = 0.025;

        public static final double min_balance_angle = 1.75;

        public static final SmartData<Double> driveKP = new SmartData("driveKP",0);
        public static final SmartData<Double> driveKI = new SmartData("driveKI",0);
        public static final SmartData<Double> driveKD = new SmartData("driveKD",0);
        public static final SmartData<Double> driveKS = new SmartData("driveKS",0);
        public static final SmartData<Double> driveKV = new SmartData("driveKV",0);

        public static final SmartData<Double> turningKP = new SmartData("turningKP",0.2);
        public static final SmartData<Double> turningKI = new SmartData("turningKI",0.15);
        public static final SmartData<Double> turningKD = new SmartData("turningKD",0.001);
        public static final SmartData<Double> turningKS = new SmartData("turningKS",0);
        public static final SmartData<Double> turningKV = new SmartData("turningKV",0);

    }

    public static enum SwerveENUMS {
        FRONT_RIGHT("front_right"),
        FRONT_LEFT("front_left"),
        BACK_RIGHT("back_right"),
        BACK_LEFT("back_left");

        public final String name;
        SwerveENUMS (String name){
            this.name = name;
        }
    }
    public static enum DriveTypeTestingENUMS {
        DRIVE_AND_TURNING("drive_and_turning"),
        DRIVE("drive"),
        TURNING("turning");

        public final String name;
        DriveTypeTestingENUMS (String name){
            this.name = name;
        }
    }
}
