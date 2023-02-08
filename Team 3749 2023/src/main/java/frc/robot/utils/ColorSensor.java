package frc.robot.utils;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import frc.robot.subsystems.Claw;

/***
 * @author Anusha Khobare
 * @author Ryan R McWeeny
 * 
 *         ColorSensor.java is color sensor code to differniate between the
 *         yellow cone and purple cube gamepieces.
 *         Note that this ultility is currently unused in claw code. But is
 *         ready for possible future use (use the gamePiece Function).
 *         Rearranged + Adapted code from RevRobotics Color Sensor Template Code
 */
public class ColorSensor {
    // defines color sensor
    private final static I2C.Port i2cPort = I2C.Port.kOnboard;
    private final static ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final static ColorMatch m_colorMatcher = new ColorMatch();

    Claw claw;

    public ColorSensor() {
        // adds colors from constants to array with all colors
        m_colorMatcher.addColorMatch(Constants.cone_color);
        m_colorMatcher.addColorMatch(Constants.cube_color);
    }

    /**
     * uses the color sensor and finds what object is in the claw currently
     * this is done by matching the object's detected color to a color stored in
     * Constants.java
     * returns the game piece that it thinks is in the claw
     * 
     * @return
     */
    public static String gamePiece() {
        // .getColor gets RGB values at the current time (what the sensor sees)
        Color detectedColor = m_colorSensor.getColor();

        // .matchClosestColor() caluclates the closest color from the listed colors in
        // Color Matcher array
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == Constants.cone_color) {
            // If cone_color detected, it is cone
            Constants.Claw.Object = "Cone";
            return Constants.Claw.Object;
        } else if (match.color == Constants.cube_color) {
            // If cube_color detected, it is cube
            Constants.Claw.Object = "Cube";
            return Constants.Claw.Object;
        } else {
            // If color detected falls out of line of margin from .matchClosestColor(), then
            // nothing is detected
            Constants.Claw.Object = "None";
            return Constants.Claw.Object;
        }

    }

    public Boolean autostop() {
        if (gamePiece() == "None") {
            claw.setSpeed(0.2);
            return false;
        } else {
            return true;
        }
    }
}