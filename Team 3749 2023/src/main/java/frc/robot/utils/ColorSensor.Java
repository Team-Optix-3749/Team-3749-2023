package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj2.Constants;
//Note that this ultility is currently unused in claw code. But is ready for possible future use (use the gamePiece Function)
//Rearranged code from RevRobotics

public class ColorSensor{
    // defines color sensor
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();

    // adds colors from constants to array with all colors
    m_colorMatcher.addColorMatch(YELLOW);
    m_colorMatcher.addColorMatch(PURPLE);

    // Intializes string stating object obtained (possible objects: "Cone", "Cube", None) is in the claw (returned in "gamePiece" method)
    public String Constants.Claw.Object;

    public static String gamePiece(){
        // .getColor gets RGB values at the current time
        Color detectedColor = m_colorSensor.getColor();
        //.matchClosestColor() caluclates the closest color (with some line of margin) from the listed colors in Color Matcher array
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == YELLOW) {
            //If yellow detected, it is yellow cone
            Constants.Claw.Object = "Cone";
        }
        else if (match.color == PURPLE) {
            //If purple detected, it is purple cube
            Constants.Claw.Object = "Cube";
        }
        else {
            //If color detected falls out of line of margin from .matchClosestColor(), then nothing is detected
            Constants.Claw.Object = None
        }
     return Object
     //return object detected (None, Cube, Cone)
    }
}