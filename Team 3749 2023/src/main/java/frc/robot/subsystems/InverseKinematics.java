package frc.robot.subsystems;

/*
 * @author Bailey Say
 * 
 * Inverse kinematics, how fun
 * 
 * Process:
 * 1. Define X and Y coordinates from length1, length2, theta1, theta 2
 * 2. Find forward kinematics (theta1, theta2) -> f(theta1, theta2) -> (x, y)
 * 3. Find inverse kinematics (x, y) -> g(x, y) -> (theta1, theta2)
 */
public class InverseKinematics {
    
    private double length1;
    private double length2;
    private double theta1;
    private double theta2;

    InverseKinematics(double length1, double length2, double theta1, double theta2) {
        this.length1 = length1;
        this.length2 = length2;
    }

    public double[] calculate(double x, double y) {
        
        return new double[2];
    }
}
