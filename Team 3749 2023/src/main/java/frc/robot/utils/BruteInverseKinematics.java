package frc.robot.utils;

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
     * @return [desired bicep angle, desired forearm angle] (radians)
     */
    public static double[] calculate(double x, double y) {

        double[] angles = new double[2];
        double minimum_distance = Double.MAX_VALUE;

        // Iterate over possible degrees for both joints
        for (double bicep_deg = 1; bicep_deg < 181; bicep_deg += 0.5) {
            double i = Math.toRadians(bicep_deg);
            for (double forearm_deg = 1; forearm_deg < 361; forearm_deg += 0.5) {
                double j = Math.toRadians(forearm_deg);

                // Change between point and current angle
                double delta_x = (Constants.Arm.bicep_length * Math.cos(i)
                        + Constants.Arm.forearm_length * Math.cos(i + j)) - x;
                double delta_y = (Constants.Arm.bicep_length * Math.sin(j)
                        + Constants.Arm.forearm_length * Math.sin(i + j)) - y;
                double distance = Math.pow(delta_x, 2) + Math.pow(delta_y, 2);

                // Once the distance surpasses the minimum_distance, exit the for loop
                if (distance >= minimum_distance)
                    break;

                minimum_distance = distance;
                angles[0] = bicep_deg;
                angles[1] = forearm_deg;
            }
        }
        return angles;
    }

}
