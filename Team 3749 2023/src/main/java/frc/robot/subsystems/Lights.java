package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.utils.Constants;
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
     * RGB Color for first light
     * @param light1
     * RGB Color for second light
     * @param light2
     * 
     * Makes a strip of two alternating colors 
     */
    public void setRGBOfStrip(int[] light1, int[] light2) {
        for (int i = 0; i < m_ledBuffer.getLength(); i += 2) {
            m_ledBuffer.setRGB(i, light1[0], light1[1], light1[2]);
            m_ledBuffer.setRGB(i + 1, light2[0], light2[1], light2[2]);
        }
    }
}


