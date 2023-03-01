package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/***
 * @author Rohin Sood
 *
 *         Wrapper class for a CommandXboxController instance
 */
public class Xbox extends CommandXboxController {

    public Xbox(final int port) {
        super(port);
    }

    public void aWhileHeld(CommandBase trueCommand) {
        super.a().whileTrue(trueCommand);
    }

    public void aWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.a().whileTrue(trueCommand).whileFalse(falseCommand);
    }

    public void aWhileHeld(Runnable whileTrue) {
        super.a().whileTrue(
                Commands.run(whileTrue));
    }

    public void aWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.a().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void aWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.a().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void aWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.a().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public void bWhileHeld(CommandBase trueCommand) {
        super.b().whileTrue(trueCommand);
    }

    public void bWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.b().whileTrue(trueCommand).whileFalse(falseCommand);
    }

    public void bWhileHeld(Runnable whileTrue) {
        super.b().whileTrue(
                Commands.run(whileTrue));
    }

    public void bWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.b().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void bWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.b().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void bWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.b().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public void xWhileHeld(CommandBase trueCommand) {
        super.x().whileTrue(trueCommand);
    }

    public void xWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.x().whileTrue(
trueCommand).whileFalse(falseCommand);
    }

    public void xWhileHeld(Runnable whileTrue) {
        super.x().whileTrue(
                Commands.run(whileTrue));
    }

    public void xWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.x().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void xWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.x().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void xWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.x().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public void yWhileHeld(CommandBase trueCommand) {
        super.y().whileTrue(
trueCommand);
    }

    public void yWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.y().whileTrue(
trueCommand).whileFalse(falseCommand);
    }

    public void yWhileHeld(Runnable whileTrue) {
        super.y().whileTrue(
                Commands.run(whileTrue));
    }

    public void yWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.y().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void yWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.y().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void yWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.y().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public void leftBumperWhileHeld(CommandBase trueCommand) {
        super.leftBumper().whileTrue(trueCommand);
    }

    public void leftBumperWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.leftBumper().whileTrue(trueCommand).whileFalse(falseCommand);
    }

    public void leftBumperWhileHeld(Runnable whileTrue) {
        super.leftBumper().whileTrue(
                Commands.run(whileTrue));
    }

    public void leftBumperWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.leftBumper().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void leftBumperWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.leftBumper().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void leftBumperWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.leftBumper().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public void rightBumperWhileHeld(CommandBase trueCommand) {
        super.rightBumper().whileTrue(trueCommand);
    }

    public void rightBumperWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.rightBumper().whileTrue(trueCommand).whileFalse(falseCommand);
    }

    public void rightBumperWhileHeld(Runnable whileTrue) {
        super.rightBumper().whileTrue(
                Commands.run(whileTrue));
    }

    public void rightBumperWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.rightBumper().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void rightBumperWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.rightBumper().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void rightBumperWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.rightBumper().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public void leftStickWhileHeld(CommandBase trueCommand) {
        super.rightBumper().whileTrue(trueCommand);
    }

    public void leftStickWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.rightBumper().whileTrue(trueCommand).whileFalse(falseCommand);
    }

    public void leftStickWhileHeld(Runnable whileTrue) {
        super.leftStick().whileTrue(
                Commands.run(whileTrue));
    }

    public void leftStickWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.leftStick().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void leftStickWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.leftStick().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void leftStickWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.leftStick().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public void leftTriggerWhileHeld(CommandBase trueCommand) {
        super.leftTrigger().whileTrue(trueCommand);
    }

    public void leftTriggerWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.leftTrigger().whileTrue(trueCommand).whileFalse(falseCommand);
    }

    public void leftTriggerWhileHeld(Runnable whileTrue) {
        super.leftTrigger().whileTrue(
                Commands.run(whileTrue));
    }

    public void leftTriggerWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.leftTrigger().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void leftTriggerWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.leftTrigger().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void leftTriggerWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.leftTrigger().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public void rightTriggerWhileHeld(CommandBase trueCommand) {
        super.rightTrigger().whileTrue(trueCommand);
    }

    public void rightTriggerWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.rightTrigger().whileTrue(trueCommand).whileFalse(falseCommand);
    }

    public void rightTriggerWhileHeld(Runnable whileTrue) {
        super.rightTrigger().whileTrue(
                Commands.run(whileTrue));
    }

    public void rightTriggerWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.rightTrigger().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void rightTriggerWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.rightTrigger().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void rightTriggerWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.rightTrigger().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public void startWhileHeld(CommandBase trueCommand) {
        super.start().whileTrue(trueCommand);
    }

    public void startWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.start().whileTrue(trueCommand).whileFalse(falseCommand);
    }

    public void startWhileHeld(Runnable whileTrue) {
        super.start().whileTrue(
                Commands.run(whileTrue));
    }

    public void startWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.start().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void startWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.start().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void startWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.start().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public void backWhileHeld(CommandBase trueCommand) {
        super.back().whileTrue(trueCommand);
    }

    public void backWhileHeld(CommandBase trueCommand, CommandBase falseCommand) {
        super.back().whileTrue(trueCommand).whileFalse(falseCommand);
    }

    public void backWhileHeld(Runnable whileTrue) {
        super.back().whileTrue(
                Commands.run(whileTrue));
    }

    public void backWhileHeld(Runnable whileTrue, Runnable whileFalse) {
        super.back().whileTrue(
                Commands.run(whileTrue))
                .whileFalse(Commands.run(whileFalse));
    }

    public void backWhileHeld(Runnable whileTrue, SubsystemBase subsystem) {
        super.back().whileTrue(
                Commands.run(whileTrue, subsystem));
    }

    public void backWhileHeld(Runnable whileTrue, Runnable whileFalse, SubsystemBase subsystem) {
        super.back().whileTrue(
                Commands.run(whileTrue, subsystem))
                .whileFalse(Commands.run(whileFalse, subsystem));
    }

    public Trigger leftBumper() {
        return super.leftBumper();
    }

    public Trigger rightBumper() {
        return super.rightBumper();
    }

    public Trigger leftStick() {
        return super.leftStick();
    }

    public Trigger rightStick() {
        return super.rightStick();
    }

    public Trigger a() {
        return super.a();
    }

    public Trigger b() {
        return super.b();
    }

    public Trigger x() {
        return super.x();
    }

    public Trigger y() {
        return super.y();
    }

    public Trigger rightTrigger() {
        return super.rightTrigger();
    }

    public Trigger leftTrigger() {
        return super.leftTrigger();
    }

    public Trigger start() {
        return super.start();
    }

    public Trigger back() {
        return super.back();
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