package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.Constants;
import frc.robot.RobotContainer;
/**
 * @author Bailey Say
 * @author Raymond Sheng (Not Don T)
 */

public class Lights extends SubsystemBase {

    public Lights() {
        AddressableLED m_led = new AddressableLED(Constants.Lights.led_port);
        AddressableLED m_ledBuffer = new AddressableLEDBuffer(60);
    }
}
