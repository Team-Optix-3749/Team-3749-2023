package frc.robot.subsystems.arm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;

// Kinematics using Tranlsation2d as vectors
// Note: uses 2D plane for now assuming orientation and target is right
public class NoahKinematics {
    // initializes origin where origin is at the bicep axle on an XY plane
    private final Translation2d origin = new Translation2d(0, 0);

    private double shoulderLength = Constants.Arm.shoulder_length;
    private double elbowLength = Constants.Arm.elbow_length;

    private double elbowParallelToShoulderAngle = 180;

    public NoahKinematics() {
    }

    // forward kinematics (can use for testing)
    // NOTE: assumes theta values are in radians
    public Translation2d forward(double thetaB, double thetaF) {
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

    
    private double lawOfCosines(double a, double b, double c){
        return Math.acos((a*a+b*b-c*c)/(2*a*b));
    }

    private double distance(double x, double y){
        return Math.sqrt(x * x +y * y);

    }
    public Pair<Double, Double> inverse(double x, double y) throws Exception{
        if (!validXYArgs(x, y)) {
            throw new Exception("invalid x and y, exceeds arm radius");
        }

        double dist = distance(x,y);
        double D_1 = Math.atan2(y,x);
        double D_2  = lawOfCosines(dist, shoulderLength, elbowLength);
        double A_1 = Math.toDegrees(D_1 + D_2);
        double A_2 = Math.toDegrees(lawOfCosines(shoulderLength, elbowLength, dist));

        A_2 -= elbowParallelToShoulderAngle;
        
        return new Pair<Double, Double>(A_1, A_2);

    }

    // validates if xy vector length doesn't exceed radius of arm
    public boolean validXYArgs(double x, double y) {
        double radiusSquared = Math.pow(shoulderLength + elbowLength, 2);
        double distanceSquared = Math.pow(x, 2) + Math.pow(y, 2);

        if (x == 0) 
            return false;

        else if (radiusSquared >= distanceSquared) 
            return true;

        return false;
    }

    // tester method
    public static void tester() throws Exception {
        ArmKinematics armKinematics = new ArmKinematics();
        Pair<Double, Double> angles = armKinematics.inverse(Math.sqrt(2) * 55, Math.sqrt(2) * 55);

        double thetaB = angles.getFirst();
        double thetaF = angles.getSecond();

        System.out.println("thetaB: " + Math.toDegrees(thetaB));
        System.out.println("thetaF: " + Math.toDegrees(thetaF));

        Translation2d endPos = armKinematics.forward(thetaB, thetaF);

        System.out.println("x-coordinate: " + endPos.getX());
        System.out.println("y-coordinate: " + endPos.getY());
    }
}
