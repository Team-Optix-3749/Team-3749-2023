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
    InverseKinematics(double length1, double length2) {
        this.length1 = length1;
        this.length2 = length2;
    }

    public double[] calculate(double x, double y) {
        
        double[] optimal = new double[2];
        double mindist = Double.MAX_VALUE;


        for (double radi = 1; radi < 91; radi++) {
            double i = Math.toDegrees(radi);
            for (double radj = 1; radj < 361; radj++) {
                double j = Math.toDegrees(radj);
                double deltax = length1 * Math.cos(i) + length2 * Math.cos(i + j) - x;
                double deltay = length1 * Math.sin(i) + length2 * Math.sin(i + j) - y;
                double distance = Math.pow(deltax, 2) + Math.pow(deltay, 2);
                if (distance < mindist)
                {
                    mindist = distance;
                    optimal[0] = i;
                    optimal[1] = j;
                }
            }
        } 
        return optimal;
    }

    public static void main(String[] args){
        InverseKinematics x = new InverseKinematics(10, 10);
        double[] smth = x.calculate(20, 20);
        System.out.println(smth[0]);
        System.out.println(smth[1]);
    }
}