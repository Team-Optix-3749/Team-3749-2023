package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;

/**
 * @author Advik Garg
 * @author William Cheng
 */
public class LimelightPhotonGetters {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision"); // New network table instance, and reading in the data table from limelight.
    
    public enum Data{
        x("tx"),
        y("ty"),
        area("ta");
        
        public final String str;

        Data(String str) {
            this.str = str;
        }    
    }
    
    public static double getDouble(Data val) { // Data val just represents the values in the networking table API (tx, ty, etc)
        return table.getEntry(val.str).getDouble(0.0); // Get the value based off the enum DATA (0.0 is how you read it in networking tables)
    }

    public static double getX() {
        double tx = getDouble(Data.x);
        return tx;
    }

    public static double getY() {
        double ty = getDouble(Data.y);
        return ty;
    }
     
    public static double getArea() {
        double area = getDouble(Data.area);
        return area;
    }
} 