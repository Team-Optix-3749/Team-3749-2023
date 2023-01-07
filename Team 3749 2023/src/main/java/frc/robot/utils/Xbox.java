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
    private JoystickButton m_leftBumper;
    private JoystickButton m_rightBumper;
    private JoystickButton m_leftStick;
    private JoystickButton m_rightStick;
    private JoystickButton m_a;
    private JoystickButton m_b;
    private JoystickButton m_x;
    private JoystickButton m_y;

    public Xbox(final int port) {
        super(port);

        m_leftBumper = new JoystickButton(this, XboxController.Button.kLeftBumper.value);
        m_rightBumper = new JoystickButton(this, XboxController.Button.kRightBumper.value);
        m_leftStick = new JoystickButton(this, XboxController.Button.kLeftStick.value);
        m_rightStick = new JoystickButton(this, XboxController.Button.kRightStick.value);
        m_a = new JoystickButton(this, XboxController.Button.kA.value);
        m_b = new JoystickButton(this, XboxController.Button.kB.value);
        m_x = new JoystickButton(this, XboxController.Button.kX.value);
        m_y = new JoystickButton(this, XboxController.Button.kY.value);

    }

    public JoystickButton leftBumper() {
        return m_leftBumper;
    }

    public JoystickButton rightBumper() {
        return m_rightBumper;
    }

    public JoystickButton leftStick() {
        return m_leftStick;
    }

    public JoystickButton rightStick() {
        return m_rightStick;
    }

    public JoystickButton a() {
        return m_a;
    }

    public JoystickButton b() {
        return m_b;
    }

    public JoystickButton x() {
        return m_x;
    }

    public JoystickButton y() {
        return m_y;
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
