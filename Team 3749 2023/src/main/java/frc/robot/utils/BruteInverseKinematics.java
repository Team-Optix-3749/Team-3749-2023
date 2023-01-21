package frc.robot.utils;

import frc.robot.utils.Constants.Arm;

/**
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
 * @author Bailey Say
 * @author Raymond Sheng
 * 
 */
public class BruteInverseKinematics {

    /**
     * Calculate the desired angle of the arm motors from the coordinate pair
     * 
     * @return [desired shoulder angle, desired elbow angle] (radians)
     */
    public static double[] calculate(double x, double y) {

        double[] angles = new double[2];
        double minimum_distance = Double.MAX_VALUE;

        // Iterate over possible degrees for both joints
        for (double shoulder_deg = Arm.shoulder_min_angle; shoulder_deg < Arm.shoulder_max_angle
                + 1; shoulder_deg += 0.5) {
            double shoulder_rad = Math.toRadians(shoulder_deg);
            for (double elbow_deg = Arm.elbow_min_angle; elbow_deg < Arm.elbow_max_angle
                    + 1; elbow_deg += 0.5) {
                double elbow_rad = Math.toRadians(elbow_deg);

                // Change between point and current angle
                double delta_x = (Constants.Arm.bicep_length * Math.cos(shoulder_rad)
                        + Constants.Arm.forearm_length * Math.cos(shoulder_rad + elbow_rad)) - x;
                double delta_y = (Constants.Arm.bicep_length * Math.sin(elbow_rad)
                        + Constants.Arm.forearm_length * Math.sin(shoulder_rad + elbow_rad)) - y;
                double distance = Math.pow(delta_x, 2) + Math.pow(delta_y, 2);

                // Once the distance surpasses the minimum_distance, exit the for loop
                if (distance >= minimum_distance)
                    break;

                minimum_distance = distance;
                angles[0] = shoulder_deg;
                angles[1] = elbow_deg;
            }
        }
        return angles;
    }

}
