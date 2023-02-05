
package frc.robot.utils;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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
    public static String Game_Piece_Identifier;

    // adds colors from constants to array with all colors
    public ColorSensor() {
        m_colorMatcher.addColorMatch(Constants.cone_color);
       // m_colorMatcher.addColorMatch(Constants.cube_color);
    }

    /**
     * Constants.Claw.Object is a String (see Constants.java)
     * States object obtained (possible objects: "Cone", "Cube", None)
     * is in the claw (returned in "gamePiece" method)
     * 
     * @return
     */
    public static String gamePiece() {

        // .getColor gets RGB values at the current time (what the sensor sees)
        Color detectedColor = m_colorSensor.getColor();

        // .matchClosestColor() caluclates the closest color from the listed colors in
        // Color Matcher array
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        SmartDashboard.setDefaultBoolean(Game_Piece_Identifier,false);
        if (match.color == Constants.cone_color) {
            // If cone_color detected, it is cone
            Game_Piece_Identifier = "Cone";
            SmartDashboard.setDefaultBoolean(Game_Piece_Identifier, true);
            // return object detected (None, Cube, Cone)
            return Game_Piece_Identifier;
        } else if (match.color == Constants.cube_color) {
            // If cube_color detected, it is cube
            Game_Piece_Identifier = "Cube";
            // return object detected (None, Cube, Cone)
            return Game_Piece_Identifier;
        } else {
            // If color detected falls out of line of margin from .matchClosestColor(), then
            // nothing is detected
            Game_Piece_Identifier = "None";
            // return object detected (None, Cube, Cone)
            return Game_Piece_Identifier;
        }

    }

    public static Boolean autostop() {
        if (gamePiece() == "None") {
            return true;
        } else {
            return false;
        }
    }

    public void AutoMotorSpeed() {
        if (autostop() == true) {
            claw.setSpeed(1.2);
        }
    }
}