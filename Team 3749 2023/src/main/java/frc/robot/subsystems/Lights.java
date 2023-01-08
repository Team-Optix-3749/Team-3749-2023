package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.Constants;
import frc.robot.RobotContainer;
/**
 * @author Bailey Say
 * @author Raymond Sheng (Not Don T)
 */

public class Lights extends SubsystemBase {

    AddressableLED m_led = new AddressableLED(Constants.Lights.led_port);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.Lights.led_length);

    public Lights() {
        
    }

    /**
     * Color for first light
     * @param r1
     * @param g1
     * @param b1
     * Color for second light
     * @param r2
     * @param g2
     * @param b2
     * 
     * Makes a strip of two alternating colors 
     */
    public void setRGBOfStrip(int r1, int g1, int b1, int r2, int g2, int b2) {
        for (int i = 0; i < m_ledBuffer.getLength(); i += 2) {
            m_ledBuffer.setRGB(i, r1, g1, b1);
            m_ledBuffer.setRGB(i + 1, r2, g2, b2);
        }
    }
}


