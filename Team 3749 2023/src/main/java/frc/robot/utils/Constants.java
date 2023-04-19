package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public static final class ArmIntake {
        public static final int arm_intake_id = 22;
        public static final double idleVoltage = 1;
        public static final double releaseConeVoltage = -8;

        public static final double intakeVoltage = 4.5;
    }

    public static final class SideIntake {
        public static final int lift_motor_id = 23;
        public static final int side_intake_id = 24;
        public static final double idleVoltage = 1.5;
        public static final double releaseObjectVoltage = -2;
        public static final double intakeVoltage = 4;

        public static final double liftKG = 0.2;
        public static final double liftKP = 7.0;

        public static final double liftOutSetpoint = 1.18;
    }

    public static final class Arm {

        // public static final double elbow_kP = 0.2;
        public static final double elbow_kP = 0.24;
        
        // public static final double shoulder_kP = 0.15;
        public static final double shoulder_kP = 0.2;

        public static final double elbow_length = 1.016;
        // public static final double elbow_cg_radius = 0.71;
        public static final double elbow_cg_radius = 0.717;
        // public static final double elbow_cg_radius = 0.714;
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
        public static final double shoulder_offset = (64-90) / 360.0;

        // just the angle offset in degrees
        public static final double elbow_offset = 41.0;

        public static final double shoulder_min_angle = 30;
        public static final double shoulder_max_angle = 140;

        public static final double elbow_min_angle = -75;
        public static final double elbow_max_angle = 260;

        public static final double maxSpeedMPS = 10;
        public static final double maxAccelerationMPS = 10;

        public static ShuffleData<Double> currWaypointX = new ShuffleData<Double>("Arm", "Current Waypoint X", 0.0);
        public static ShuffleData<Double> currWaypointY = new ShuffleData<Double>("Arm", "Current Waypoint Y", 0.0);
        public static ShuffleData<Double> armCoordinateX = new ShuffleData<Double>("Arm", "Arm Coordinate X (Move arm)",
                0.0);
        public static ShuffleData<Double> armCoordinateY = new ShuffleData<Double>("Arm", "Arm Coordinate Y (Move arm)",
                0.0);

        public static enum ArmSetpoints {
            STOW(new Translation2d(0.43, -0.2)),
            // CUBE_STOW(new Translation2d(0.381, 0.0348)),  LEGAL
            CUBE_STOW(new Translation2d(0.45, 0.1)),
            STING(new Translation2d(0.65, 0.7)),
            DOUBLE_SUBSTATION_CUBE(new Translation2d(0.5, 0.9)),
            DOUBLE_SUBSTATION_CONE(new Translation2d(0.5, 0.83)),
            SINGLE_SUBSTATION(new Translation2d(0.5, 0.5)), // NOT REAL
            // GROUND_INTAKE_CUBE(new Translation2d(1.19, -0.14)),
            GROUND_INTAKE_CUBE(new Translation2d(1.19, -0.115)),

            PLACE_TOP(new Translation2d(1.215, 1.05)),
            PLACE_MID(new Translation2d(0.85, 0.73));

            public Translation2d translation;

            ArmSetpoints(Translation2d translation) {
                this.translation = translation;
            }

            public Pose2d toPose2d(double rotation) {
                return new Pose2d(this.translation, new Rotation2d(rotation));
            }
        }
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5);
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

        public static double kTeleDriveMaxSpeedMetersPerSecond = 6;
        public static double kTeleDriveMaxAngularSpeedRadiansPerSecond = 7;
        public static double kTeleDriveMaxAccelerationUnitsPerSecond = 6;
        public static double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 7;

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
        public static final double max_yaw_offset = 13;
        public static final double max_pitch_offset = 2;
        public static final double max_pitch_margin = 3;
        public static final double max_movement_offset = 0.025; // around 1 inch

        public static final double base_speed_mps = 0.8;

        public static final double adjust_speed_mps = 0.1;
        public static final double adjusting_distance = 0.05; // In meters, so this is 5 cm
        public static final double adjust_time_length = 0.2; // in seconds
    }

    public static final class AutoConstants {
        public static final Map<String, Command> eventMap = new HashMap<>();

        public static enum TopBottom {
            TOP,
            BOTTOM;
        }
    }

    public static class VisionConstants {
        // See
        // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
        // page 208
        public static final double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

        // See
        // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
        // page 197
        public static final double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

        public static final Transform3d cam_to_robot = new Transform3d(
                new Translation3d(0, 0, -Units.inchesToMeters(15.25)), new Rotation3d());
        public static final Transform3d robot_to_cam = cam_to_robot.inverse();

        public static final int reflective_tape_pipeline_index = 0;
        public static final int apriltag_pipeline_index = 1;

        public static final double camera_height = Units.inchesToMeters(20); // meters
        public static final double camera_yaw = 0;
        public static final double camera_pitch = 0;
        // public static final double camera_pitch = -2.66 ;

        public static final double retro_cam_offset = 0.56;
        public static final double apriltag_cam_offset = 3.1;

        public static final ShuffleData<Double> goalPoseX = new ShuffleData<Double>("Limelight", "Goal Pose X",
                -1000.0);
        public static final ShuffleData<Double> goalPoseY = new ShuffleData<Double>("Limelight", "Goal Pose Y",
                -1000.0);
        public static final ShuffleData<Double> goalPoseHeading = new ShuffleData<Double>("Limelight",
                "Goal Pose Heading", -1000.0);

        public static enum Node {
            CONE(0), CUBE(Units.inchesToMeters(14.25)), MID_CONE(24), TOP_CONE(43);

            public double height_meters;

            Node(double height_meters) {
                this.height_meters = height_meters;
            }
        };

        public static enum Piece {
            CONE, CUBE;
        }

        public static enum Pipelines {
            APRILTAG(1),
            CUBE(0);
            
            public int index;

            Pipelines(int index) {
                this.index = index;
            }
        }
    }

    public static class LEDs {
        public static final int pwm_port = 9;
        public static final int length = 92;

        public static enum LEDPattern {
            RAINBOW, RED, BLUE, GREEN, PURPLE, YELLOW, WHITE, BOUNCE, BLINK, TWINKLE, NOTHING
        };
    }
}