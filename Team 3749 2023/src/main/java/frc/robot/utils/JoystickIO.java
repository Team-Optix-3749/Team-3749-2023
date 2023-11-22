package frc.robot.utils;

import java.util.Map;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.subsystems.swerve.Swerve;


/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 */
public class JoystickIO {
    private static String[] lastJoystickNames = new String[] { "", "", "", "", "", "" };

    private Xbox pilot;
    private Xbox operator;

    private Swerve swerve;
 
    public JoystickIO(Xbox pilot, Xbox operator) {
        this.pilot = pilot;
        this.operator = operator;
        this.swerve = Robot.swerve;
    }

    /**
     * Calls binding methods according to the joysticks connected
     */
    public void getButtonBindings() {



        setDefaultCommands();
    }

    /**
     * Sets the default commands
     */
    public void setDefaultCommands() {
        swerve.setDefaultCommand(new SwerveTeleopCommand(

                () -> -pilot.getLeftY(),
                () -> -pilot.getLeftX(),
                () -> pilot.getRightX()));

    }
}
