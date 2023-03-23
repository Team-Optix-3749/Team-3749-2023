package frc.robot.subsystems.leds;

import java.util.Random;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.LEDs.LEDPattern;

/**
 * Class for controlling addressable LEDs
 * 
 * @author Rohin Sood
 */
public class LEDs extends SubsystemBase {
    private LEDPattern currentLEDPattern = getDefaultColor();
    private AddressableLED LEDs = new AddressableLED(Constants.LEDs.pwm_port);
    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(Constants.LEDs.length);
    private int hue = 0;
    private int i = 0;
    private boolean blink = false;
    
    public LEDs(){ 
        LEDs.setLength(LEDBuffer.getLength());
        LEDs.setData(LEDBuffer);
        LEDs.start();
    }

    public LEDPattern getDefaultColor() {
        return DriverStation.getAlliance() == Alliance.Blue ? LEDPattern.BLUE : LEDPattern.RED;
    }

    /**
     * Set RGB values
     * 
     * @param red
     * @param green
     * @param blue
     */
    private void setRGB(int red, int green, int blue){
        for(int led = 0; led < LEDBuffer.getLength(); led++){
            LEDBuffer.setRGB(led, red, green, blue);
        }
    }

    /**
     * Set HSV values
     * 
     * @param hue
     * @param saturation
     * @param value
     */
    private void setHSV(int hue, int saturation, int value){
        for(int led = 0; led < LEDBuffer.getLength(); led++){
            LEDBuffer.setHSV(led, hue, saturation, value);
        }
    }

    private void rainbowLEDs(){
        setHSV(hue++, 255, 255);
        if(hue >= 180){
            hue = 0;
        }          
    }

    private void twinkleLEDs(){
        Random rand = new Random();
        int led_max = LEDBuffer.getLength();
        
        /* Set all to off */
        for(int i = 0; i < led_max; i++){
            LEDBuffer.setHSV(i, 100, 255, 20);
        }

        /* Grab random led ten times */
        for(int i = 0; i < 30; i++){
            /* Set to lower value */
            LEDBuffer.setRGB(rand.nextInt(led_max), 0, 255, 0);
        }
    }

    private void bounceLEDs(){
        int led_max = LEDBuffer.getLength();
        if(i == led_max){
           i = 0;
           return;
        }
            
        if(i != 0){
            LEDBuffer.setHSV(i-1, 100, 255, 0);
            i++;
            return;
        }
            
        LEDBuffer.setHSV(i, 100, 255, 255);
        i++;
    }

    private void blinkLEDs() {
        int led_max = LEDBuffer.getLength();

        for (int i= 0; i < led_max; i++) {
            if (blink) {
                setLEDPattern(LEDPattern.GREEN);
            } else {
                setHSV(0, 255, 255);
            }
        }

        blink = !blink;
    }

    /**
     * setter for the current LEDsPattern
     * 
     * @param pattern
     */
    public void setLEDPattern(LEDPattern pattern){
        this.currentLEDPattern = pattern;
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic(){
        switch(currentLEDPattern){
            case RAINBOW:
                // not checking if already in this state because the rainbow moves in a wave pattern
                rainbowLEDs();
                break;
            case RED:
                    setRGB(255, 0, 0);
                    this.setLEDPattern(LEDPattern.RED);
                    break;
                
            case GREEN: 
                    // Red Hue: 170-15 
                    setRGB(0, 255, 0);
                    this.setLEDPattern(LEDPattern.GREEN);
                    break;
                
            case BLUE:
                    // Green Hue: 40-75
                    setRGB(0, 0, 255);
                    this.setLEDPattern(LEDPattern.BLUE);
                    break;

            case WHITE:
                    // Green Hue: 40-75
                    setRGB(100, 100, 100);
                    this.setLEDPattern(LEDPattern.WHITE);
                    break;

            case BOUNCE:
                if (currentLEDPattern != LEDPattern.BOUNCE) i = 0;
            
                bounceLEDs();
                break;
            case BLINK:

                blinkLEDs();
                break;
            
            case TWINKLE:
                /* setLEDSTwinkle does NOT set the value once, and must be updated periodically */
                twinkleLEDs();
                break;
            case NOTHING:
                break;
            default: 
                setRGB(180, 255, 255);
                System.out.println("ERROR: LEDs switch case not getting a color pattern");
                break;   
        }

        LEDs.setData(LEDBuffer);
    }
    
}