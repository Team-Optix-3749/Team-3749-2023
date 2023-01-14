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
 * 
 * Unfortunately original plan was scrapped
 * Now just do brute force to find all solutions and optimize solution
 */
public class InverseKinematics {
    
    private double length1;
    private double length2;
    InverseKinematics(double length1, double length2, double theta1, double theta2) {
        this.length1 = length1;
        this.length2 = length2;
    }

    public int[] calculate(double x, double y) {
        
        int[] optimal = new int[2];

        // Iterate over 1-360 degrees for both joints
        for (int i = 1; i < 361; i++) {
            for (int j = 1; j < 361; j++) {
                if (x == length1 * Math.cos(i) + length2 * Math.cos(i + j) && y == length1 * Math.sin(i) + length2 * Math.sin(i + j)) {
                    
                }
            }
        } 
        return optimal;
    }
}
