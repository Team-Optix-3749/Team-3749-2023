package frc.robot.subsystems.led;

import java.util.Random;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.LEDs.LEDsPattern;

/**
 * Class for controlling addressable LEDs
 * 
 * @author Rohin Sood
 */
public class LEDs extends SubsystemBase {
    private LEDsPattern currentLEDsPattern = LEDsPattern.GREEN;
    private AddressableLED LED = new AddressableLED(Constants.LEDs.pwm_port);
    private AddressableLEDBuffer LEDsBuffer = new AddressableLEDBuffer(Constants.LEDs.length);
    private int hue = 0;
    private int i = 0;
    
    public LEDs(){ 
        LED.setLength(LEDsBuffer.getLength());
        LED.setData(LEDsBuffer);
        LED.start();
    }

    private void setLEDsRainbow(){
        setLEDsColor(hue++, 255, 255);
        if(hue >= 180){
            hue = 0;
        }          
    }

    /**
     * Change LED colors in rainbow for launch, regular run
     * 
     * @param hue
     * @param saturation
     * @param value
     */
    private void setLEDsColor(int hue, int saturation, int value){
        for(int led = 0; led < LEDsBuffer.getLength(); led++){
            LEDsBuffer.setHSV(led, hue, saturation, value);
        }
    }

    private void setLEDsTwinkle(){
        Random rand = new Random();
        int led_max = LEDsBuffer.getLength();
        
        /* Set all to off */
        for(int i = 0; i < led_max; i++){
            LEDsBuffer.setHSV(i, 100, 255, 20);
        }

        /* Grab random led ten times */
        for(int i = 0; i < 30; i++){
            /* Set to lower value */
            LEDsBuffer.setHSV(rand.nextInt(led_max),100, 255, rand.nextInt(255));
        }
    }

    private void setLEDsBounce(){
        int led_max = LEDsBuffer.getLength();
        if(i == led_max){
           LEDsBuffer.setHSV(i, 100, 255, 0);
           i = 0;
           return;
        }
            
        if(i != 0){
            LEDsBuffer.setHSV(i-1, 100, 255, 0);
            i++;
            return;
        }
            
        LEDsBuffer.setHSV(i, 100, 255, 255);
        i++;
    }

    /**
     * setter for the current LEDsPattern
     * 
     * @param pattern
     */
    public void setLEDsPattern(LEDsPattern pattern){
        currentLEDsPattern = pattern;
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic(){
        switch(currentLEDsPattern){
            case RAINBOW:
                // not checking if already in this state because the rainbow moves in a wave pattern
                setLEDsRainbow();
                break;
            case BLUE:
                if (currentLEDsPattern != LEDsPattern.BLUE) {
                    // Blue Hue: 85-135
                    setLEDsColor(100, 255, 255);
                    this.setLEDsPattern(LEDsPattern.BLUE);
                    break;
                }
                
            case RED: 
                if (currentLEDsPattern != LEDsPattern.RED) {
                    // Red Hue: 170-15 
                    setLEDsColor(180, 255, 255);
                    this.setLEDsPattern(LEDsPattern.RED);
                    break;
                }
            case GREEN:
                if (currentLEDsPattern != LEDsPattern.GREEN) {
                    // Green Hue: 40-75
                    setLEDsColor(50, 255, 255);
                    this.setLEDsPattern(LEDsPattern.GREEN);
                    break;
                }

            case WHITE:
                if (currentLEDsPattern != LEDsPattern.WHITE) {
                    // Green Hue: 40-75
                    setLEDsColor(200, 33, 98);
                    this.setLEDsPattern(LEDsPattern.WHITE);
                    break;
                }
            case BOUNCE:
                setLEDsBounce();
                break;
            case TWINKLE:
                /* setLEDSTwinkle does NOT set the value once, and must be updated periodically */
                setLEDsTwinkle();
                break;
            case NOTHING:
                break;
            default: 
                setLEDsColor(180, 255, 255);
                System.out.println("ERROR: LEDs switch case Not getting a color pattern");
                break;   
        }
        // Sends the LED buffer data to the LEDS 
        LED.setData(LEDsBuffer);
    }
}