package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj2.Constants;

public class ColorSensor{
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    m_colorMatcher.addColorMatch(YELLOW);
    m_colorMatcher.addColorMatch(PURPLE);
    public String Object;

    public static gamePiece(){
        Color detectedColor = m_colorSensor.getColor();

        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == YELLOW) {
            Object = "Cone";
    }   else if (match.color == PURPLE) {
            Object = "Cube";
    }   else {
            Object = None
    }
     return Object
    }

    
}