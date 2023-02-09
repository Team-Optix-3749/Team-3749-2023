package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/***
 * @author Rohan Juneja
 * @author Rohin Sood
 * 
 *         Stores getters for all Xbox controller inputs
 */
public class Xbox extends GenericHID {
    private JoystickButton leftBumper;
    private JoystickButton rightBumper;
    private JoystickButton leftStick;
    private JoystickButton rightStick;
    private JoystickButton a;
    private JoystickButton b;
    private JoystickButton x;
    private JoystickButton y;

    public Xbox(final int port) {
        super(port);

        leftBumper = new JoystickButton(this, XboxController.Button.kLeftBumper.value);
        rightBumper = new JoystickButton(this, XboxController.Button.kRightBumper.value);
        leftStick = new JoystickButton(this, XboxController.Button.kLeftStick.value);
        rightStick = new JoystickButton(this, XboxController.Button.kRightStick.value);
        a = new JoystickButton(this, XboxController.Button.kA.value);
        b = new JoystickButton(this, XboxController.Button.kB.value);
        x = new JoystickButton(this, XboxController.Button.kX.value);
        y = new JoystickButton(this, XboxController.Button.kY.value);

    }

    public JoystickButton leftBumper() {
        return leftBumper;
    }

    public JoystickButton rightBumper() {
        return rightBumper;
    }

    public JoystickButton leftStick() {
        return leftStick;
    }

    public JoystickButton rightStick() {
        return rightStick;
    }

    public JoystickButton a() {
        return a;
    }

    public JoystickButton b() {
        return b;
    }

    public JoystickButton x() {
        return x;
    }

    public JoystickButton y() {
        return y;
    }

    public double getLeftX() {
        return getRawAxis(XboxController.Axis.kLeftX.value);
    }

    public double getRightX() {
        return getRawAxis(XboxController.Axis.kRightX.value);
    }

    public double getLeftY() {
        return getRawAxis(XboxController.Axis.kLeftY.value);
    }

    public double getRightY() {
        return getRawAxis(XboxController.Axis.kRightY.value);
    }

    public boolean getLeftTrigger() {
        return getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.1;
    }

    public boolean getRightTrigger() {
        return getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.1;
    }
}
