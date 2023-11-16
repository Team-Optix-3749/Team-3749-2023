package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;


/***
 * @author Rohan Juneja
 * @author Rohin Sood
 * 
 *         Stores data that can be seen/edited in Smart Dashboard
 * @param <T> Type of data: Supported types include String, Double, and Boolean
 */
public class SmartData<T> {
    private T defaultVal;
    private NetworkTableEntry entry;
    private Map<Integer, T> lastHasChangedVals = new HashMap<>();

    /**
     * Creates a new SmartData instance
     * 
     * @param name       Key on SmartDashboard
     * @param defaultVal Default value
     */
    public SmartData(String name, T defaultVal) {
        this.defaultVal = defaultVal;
        entry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry(name);
        entry.setValue(defaultVal);
    }

    public void setDefault(T defaultVal) {
        this.defaultVal = defaultVal;
    }

    @SuppressWarnings("unchecked")
    public T get() {
        NetworkTableValue value = entry.getValue();

        if (value.getType() == NetworkTableType.kBoolean) {
            return (T) (Boolean) value.getBoolean();

        } else if (value.getType() == NetworkTableType.kDouble) {
            return (T) (Double) value.getDouble();

        } else if (value.getType() == NetworkTableType.kString) {
            return (T) (String) value.getString();

        } else {
            return this.defaultVal;
        }
    }

    /**
     * checks whether the number has changed since the last time this method has
     * been called
     * 
     * @param id Unique identified for the called to avoid conflicts shared between
     *           multiple instances. Recommended to pass in the result of
     *           'Obj.hashCode()'
     * @return true if the value has changed for its last check
     */
    public boolean hasChanged(int id) {
        T currentVal = get();
        T lastVal = lastHasChangedVals.get(id);

        if (lastVal == null || !currentVal.equals(lastVal)) {
            lastHasChangedVals.put(id, currentVal);
            return true;
        }

        return false;
    }

    public void set(T val) {
        entry.setValue(val);
    }
}
