package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/***
 * @author Rohin Sood
 * 
 *         Stores getters for all CommandXboxControllers outputs (rips .whenPressed())
 */
public class Xbox extends CommandXboxController {
    private Trigger leftBumper;
    private Trigger rightBumper;
    private Trigger leftStick;
    private Trigger rightStick;
    private Trigger a;
    private Trigger b;
    private Trigger x;
    private Trigger y;

    public Xbox(final int port) {
        super(port);

        leftBumper = this.leftBumper();
        rightBumper = this.rightTrigger();
        leftStick = this.leftStick();
        rightStick = this.rightStick();
        a = this.a();
        b = this.b();
        x = this.x();
        y = this.y();
    }

    public void aWhenPressed(Runnable onTrue, SubsystemBase subsystem) {
        a.onTrue(
                Commands.run(onTrue, subsystem));
    }

    public void aWhenPressed(Runnable onTrue, Runnable onFalse, SubsystemBase subsystem) {
        a.onTrue(
                Commands.run(onTrue, subsystem))
                .onFalse(Commands.run(onFalse, subsystem));
    }

    public void bWhenPressed(Runnable onTrue, SubsystemBase subsystem) {
        b.onTrue(
                Commands.run(onTrue, subsystem));
    }

    public void bWhenPressed(Runnable onTrue, Runnable onFalse, SubsystemBase subsystem) {
        b.onTrue(
                Commands.run(onTrue, subsystem))
                .onFalse(Commands.run(onFalse, subsystem));
    }

    public void xWhenPressed(Runnable onTrue, SubsystemBase subsystem) {
        x.onTrue(
                Commands.run(onTrue, subsystem));
    }

    public void xWhenPressed(Runnable onTrue, Runnable onFalse, SubsystemBase subsystem) {
        x.onTrue(
                Commands.run(onTrue, subsystem))
                .onFalse(Commands.run(onFalse, subsystem));
    }

    public void yWhenPressed(Runnable onTrue, SubsystemBase subsystem) {
        y.onTrue(
                Commands.run(onTrue, subsystem));
    }

    public void yWhenPressed(Runnable onTrue, Runnable onFalse, SubsystemBase subsystem) {
        y.onTrue(
                Commands.run(onTrue, subsystem))
                .onFalse(Commands.run(onFalse, subsystem));
    }

    public void leftBumperWhenPressed(Runnable onTrue, SubsystemBase subsystem) {
        leftBumper.onTrue(
                Commands.run(onTrue, subsystem));
    }

    public void leftBumperWhenPressed(Runnable onTrue, Runnable onFalse, SubsystemBase subsystem) {
        leftBumper.onTrue(
                Commands.run(onTrue, subsystem))
                .onFalse(Commands.run(onFalse, subsystem));
    }

    public void rightBumperWhenPressed(Runnable onTrue, SubsystemBase subsystem) {
        rightBumper.onTrue(
                Commands.run(onTrue, subsystem));
    }

    public void rightBumperWhenPressed(Runnable onTrue, Runnable onFalse, SubsystemBase subsystem) {
        rightBumper.onTrue(
                Commands.run(onTrue, subsystem))
                .onFalse(Commands.run(onFalse, subsystem));
    }

    public Trigger leftBumper() {
        return leftBumper;
    }

    public Trigger rightBumper() {
        return rightBumper;
    }

    public Trigger leftStick() {
        return leftStick;
    }

    public Trigger rightStick() {
        return rightStick;
    }

    public Trigger a() {
        return a;
    }

    public Trigger b() {
        return b;
    }

    public Trigger x() {
        return x;
    }

    public Trigger y() {
        return y;
    }

    public double getLeftX() {
        return this.getRawAxis(XboxController.Axis.kLeftX.value);
    }

    public double getRightX() {
        return this.getRawAxis(XboxController.Axis.kRightX.value);
    }

    public double getLeftY() {
        return this.getRawAxis(XboxController.Axis.kLeftY.value);
    }

    public double getRightY() {
        return this.getRawAxis(XboxController.Axis.kRightY.value);
    }

    public boolean getLeftTrigger() {
        return this.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.1;
    }

    public boolean getRightTrigger() {
        return this.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.1;
    }

}
