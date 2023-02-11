package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

// Kinematics using Tranlsation2d as vectors
// Note: uses 2D plane for now assuming orientation and target is right
public class Kinematics {
    // initializes origin where origin is at base of robot on an XY plane
    private final Translation2d origin = new Translation2d(0,0);
    
    private double bicepLength = Constants.Arm.bicep_length;
    private double forearmLength = Constants.Arm.forearm_length;

    // "zero" position in terms of angle set to directly vertical
    private Translation2d bicepZero = new Translation2d(0, bicepLength);
    private Translation2d forearmZero = new Translation2d(0, forearmLength);
    
    public Kinematics(double xi, double yi){ 
        // initial positions of (x,y) for arm
        // TODO: set positions to default
    }

    // forward kinematics (can use for testing)
    // NOTE: assumes theta values are in radians
    public Translation2d forward(double thetaB, double thetaF){
        Translation2d bicepVector = new Translation2d(
                bicepLength * Math.cos(thetaB), 
                bicepLength * Math.sin(thetaB));

        Translation2d forearmVector = new Translation2d(
            forearmLength * Math.cos(thetaB-thetaF), 
            forearmLength * Math.sin(thetaB-thetaF));

        Translation2d finalPositionVector = origin
            .plus(forearmVector)
            .plus(bicepVector);

        return finalPositionVector;
    }

    // for now returns a pair of doubles, might need to change later
    // needs an algebraic solver
    public Pair<Double, Double> inverse(double x, double y) throws Exception{
        if(!validXYArgs(x, y)){
            throw new Exception("invalid x and y, exceeds arm radius");
        }
        
        // final vector location
        Translation2d finalPos = new Translation2d(x, y);

        Double thetaF = -Math.acos(
            (Math.pow(x, 2) +
            Math.pow(y, 2) -
            Math.pow(bicepLength, 2) -
            Math.pow(forearmLength, 2)) / 
            (2 * bicepLength * forearmLength)
        );

        Double thetaB = Math.atan(y/x) + 
            Math.atan(
                (forearmLength*Math.sin(thetaF))/(bicepLength + forearmLength*Math.cos(thetaF))
            );

        return new Pair<Double, Double>(thetaB, thetaF);
    }

    // validates xy vector lengths don't exceed radius of arm
    public boolean validXYArgs(double x, double y){
        double radiusSquared = Math.pow(bicepLength + forearmLength, 2);
        double distanceSquared = Math.pow(x, 2) + Math.pow(y, 2);

        if(radiusSquared >= distanceSquared){
            return true;
        }

        return false;
    }

    public static void tester() throws Exception {
        Kinematics kinematics = new Kinematics(0, 0);
        Pair<Double, Double> angles = kinematics.inverse(55/Math.sqrt(2), 55/Math.sqrt(2));

        double thetaB = angles.getFirst();
        double thetaF = angles.getSecond();

        System.out.println("thetaB: " + Math.toDegrees(thetaB));
        System.out.println("thetaF: " + Math.toDegrees(thetaF));

        Translation2d endPos = kinematics.forward(thetaB, thetaF);

        System.out.println("x-coordinate: " + endPos.getX());
        System.out.println("y-coordinate: " + endPos.getY());
    }
}
