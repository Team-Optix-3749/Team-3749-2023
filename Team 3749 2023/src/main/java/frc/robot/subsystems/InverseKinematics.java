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
        
        // Define XE_RHS and YE_RHS as a function of lengths and thetas
        double XE_RHS = length1 * Math.cos(this.theta1) + length2 * Math.cos(this.theta1 + this.theta2);
        double YE_RHS = length1 * Math.sin(this.theta1) + length2 * Math.sin(this.theta1 + this.theta2);
        return new double[2];
    }
}
