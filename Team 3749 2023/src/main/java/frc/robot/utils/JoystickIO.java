package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 */
public class JoystickIO {
    private static String[] lastJoystickNames = new String[] { "", "", "", "", "", "" };

    private Xbox pilot;

    private Swerve swerve;

    public JoystickIO(Xbox pilot, Swerve swerve) {
        this.pilot = pilot;
        this.swerve = swerve;
    }

    public static boolean didJoysticksChange() {
        boolean joysticksChanged = false;
        for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
            String name = DriverStation.getJoystickName(port);
            if (!name.equals(lastJoystickNames[port])) {
                joysticksChanged = true;
                lastJoystickNames[port] = name;
            }
        }
        return joysticksChanged;
    }

    /**
     * Calls binding methods according to the joysticks connected
     */
    public void getButtonBindings() {

        if (DriverStation.isJoystickConnected(1)) {
            // if both xbox controllers are connected
            pilotBindings();

        } else if (DriverStation.isJoystickConnected(0)) {
            // if only one xbox controller is connected
            pilotBindings();

        } else {
            // if no joysticks are connected (ShuffleBoard buttons)
            pilotBindings();

        }
    }

    /**
     * If only one controller is plugged in (pi)
     */
    public void pilotBindings() {
        pilot.yWhileHeld(() -> swerve.toggleSpeed());
        
        pilot.startWhileHeld(Commands.runOnce(() -> {
            swerve.unflipGyro();
            swerve.resetGyro();
        }, swerve));
        pilot.backWhileHeld(Commands.runOnce(() -> {
            swerve.unflipGyro();
            swerve.resetGyro();
        }, swerve));

        // swerve rotation cardinals
        pilot.povUp().whileTrue(Commands.run(() -> swerve.turnToRotation(0)));
        pilot.povLeft().whileTrue(Commands.run(() -> swerve.turnToRotation(270)));
        pilot.povDown().whileTrue(Commands.run(() -> swerve.turnToRotation(180)));
        pilot.povRight().whileTrue(Commands.run(() -> swerve.turnToRotation(90)));
    }

}
