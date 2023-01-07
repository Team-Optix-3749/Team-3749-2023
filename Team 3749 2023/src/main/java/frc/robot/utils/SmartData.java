package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;

/***
 * @author Rohan Juneja
 * @author Rohin Sood
 * 
 * Stores data that can be seen/edited in Smart Dashboard
 * @param <T> Type of data: Supported types include String, Double, and Boolean
 */
public class SmartData<T> {
    T defaultVal;
    NetworkTableEntry entry;

    public SmartData (String name, T defaultVal) {
        this.defaultVal = defaultVal;
        entry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry(name);
        entry.setValue(defaultVal);
    }

    public void setDefault (T defaultVal) {
        this.defaultVal = defaultVal;
    }

    @SuppressWarnings("unchecked") 
    public T get() {
        NetworkTableValue value = entry.getValue();

        if (value.getType() == NetworkTableType.kBoolean) {
            return  (T) (Boolean) value.getBoolean();

        } else if (value.getType() == NetworkTableType.kDouble) {
            return  (T) (Double) value.getDouble();

        } else if (value.getType() == NetworkTableType.kString) {
            return  (T) (String) value.getString();
            
        } else {
            return this.defaultVal;
        }
    }
    
    public void set(T val) {
        entry.setValue(val);
    }
}
