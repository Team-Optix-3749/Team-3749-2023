package frc.robot.subsystems.arm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.Constants;

/***
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 * 
 * {@link} https://appliedgo.net/roboticarm/
 * Kinematics using Tranlsation2d as vectors
 */
public class ArmKinematics {
    // initializes origin where origin is--at the bicep axle--on an XY plane
    private static final Translation2d origin = new Translation2d(0, 0);

    private static double shoulderLength = Constants.Arm.shoulder_length;
    private static double elbowLength = Constants.Arm.elbow_length;

    private static double elbowParallelToShoulderAngle = 180;

    /***
     * @param thetaB Bicep angle in radians
     * @param thetaF forearm angle in radians
     * @return Translation2d the coordinate position of the end of the arm, in meters
     */
    public static Translation2d forward(double thetaB, double thetaF) {
        Translation2d bicepVector = new Translation2d(
                shoulderLength * Math.cos(thetaB),
                shoulderLength * Math.sin(thetaB));

        Translation2d forearmVector = new Translation2d(
                elbowLength * Math.cos(thetaB + thetaF),
                elbowLength * Math.sin(thetaB + thetaF));

        Translation2d finalPositionVector = origin
                .plus(forearmVector)
                .plus(bicepVector);

        return finalPositionVector;
    }

    private static double lawOfCosines(double a, double b, double c) {
        return Math.acos((a * a + b * b - c * c) / (2 * a * b));
    }

    private static double distance(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }
    /***
     * @param x horizontal translation in meters
     * @param y vertical translation in meters
     * @return Pair<Double, Double> the bicep angle in degrees is first, the forearm angle in degrees is second
     * @throws Exception if the coordinate is outside of the possible range of the arm
     */
    public static Pair<Double, Double> inverse(double x, double y) throws Exception {
        if (!validXYArgs(x, y)) {
            throw new Exception("invalid x and y, exceeds arm radius");
        }
        double dist = distance(x, y);
        // the adjacent angle of the right triangle drawn from the origin to the x-y coordinate
        double D_1 = Math.atan2(y, x);
        // the angle next to D_1 that is part of the triangle drawn from the origin to the bicep-forearm connection to the x-y coordinate
        double D_2 = lawOfCosines(dist, shoulderLength, elbowLength);
        // the D's combine to form the total bicep angle
        double A_1 = Math.toDegrees(D_1 + D_2);
        // The forearm angle is based on the law of Cosines 
        double A_2 = Math.toDegrees(lawOfCosines(shoulderLength, elbowLength, dist));
        // adjust so that the elbow/forearm is at 0 degrees when in line with the shoulder axles 
        A_2 -= elbowParallelToShoulderAngle;

        return new Pair<Double, Double>(A_1, A_2);

    }

    // validates if xy vector length doesn't exceed radius of arm
    public static boolean validXYArgs(double x, double y) {
        double radiusSquared = Math.pow(shoulderLength + elbowLength, 2);
        double distanceSquared = Math.pow(x, 2) + Math.pow(y, 2);

        if (x == 0)
            return false;

        else if (radiusSquared >= distanceSquared)
            return true;

        return false;
    }
}
