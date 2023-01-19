package frc.robot.subsystems;

import frc.robot.utils.Constants;

/*
 * @author Bailey Say
 * 
 * Inverse kinematics, how fun
 * It actually this is more like trig :skull:
 * 
 * Process:
 * 1. Define X and Y coordinates from length1, length2, theta1, theta 2
 * 2. Find forward kinematics (theta1, theta2) -> f(theta1, theta2) -> (x, y)
 * 3. Find inverse kinematics (x, y) -> g(x, y) -> (theta1, theta2)
 * 
 * Unfortunately original plan was scrapped
 * Now just do brute force to find all solutions and optimize solution
 * 
 */
public class InverseKinematics {
    
    public static double[] calculate(double x, double y) {
        
        double[] optimal = new double[2];
        double mindist = Double.MAX_VALUE;

        // Iterate over 1-360 degrees for both joints
        for (double degi = 1; degi < 181; degi += 0.5) {
            double i = Math.toRadians(degi);
            for (double degj = 1; degj < 361; degj += 0.5) {
                double j = Math.toRadians(degj);
                double deltax = Constants.Arm.lower_length * Math.cos(i) + Constants.Arm.upper_length * Math.cos(i + j) - x; 
                double deltay = Constants.Arm.lower_length * Math.sin(j) + Constants.Arm.upper_length * Math.sin(i + j) - y;
                double distance = Math.pow(deltax, 2) + Math.pow(deltay, 2);
                if (distance < mindist)
                {
                    mindist = distance;
                    optimal[0] = degi;
                    optimal[1] = degj;
                }
            }
        } 
        return optimal;
    }

    public static void main(String[] args) {
        double[] solution1 = calculate(20,20);
        System.out.println(solution1[0] + " " + solution1[1]);
    }
}
