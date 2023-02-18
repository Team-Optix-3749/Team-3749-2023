package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/***
 * @author Rohan Juneja
 * @author Rohin Sood
 * 
 *         Stores getters for all POV buttons
 */
public class POV {
    private final GenericHID hid;
    private final int povNumber;

    private POVButton upButton; // 0 degrees
    private POVButton upRightButton; // 45 degrees
    private POVButton rightButton; // 90 degrees
    private POVButton downRightButton; // 135 degrees
    private POVButton downButton; // 180 degrees
    private POVButton downLeftButton; // 225 degrees
    private POVButton leftButton; // 270 degrees
    private POVButton upLeftButton; // 315 degrees

    public POV(GenericHID hid) {
        this(hid, 0);
    }

    public POV(GenericHID hid, int povNumber) {
        this.hid = hid;
        this.povNumber = povNumber;
    }

    public POVButton up() {
        if (upButton == null) {
            upButton = new POVButton(hid, 0, povNumber);
        }

        return upButton;
    }

    public POVButton upRight() {
        if (upRightButton == null) {
            upRightButton = new POVButton(hid, 45, povNumber);
        }

        return upRightButton;
    }

    public POVButton right() {
        if (rightButton == null) {
            rightButton = new POVButton(hid, 90, povNumber);
        }

        return rightButton;
    }

    public POVButton downRight() {
        if (downRightButton == null) {
            downRightButton = new POVButton(hid, 135, povNumber);
        }

        return downRightButton;
    }

    public POVButton down() {
        if (downButton == null) {
            downButton = new POVButton(hid, 180, povNumber);
        }

        return downButton;
    }

    public POVButton downLeft() {
        if (downLeftButton == null) {
            downLeftButton = new POVButton(hid, 225, povNumber);
        }

        return downLeftButton;
    }

    public POVButton left() {
        if (leftButton == null) {
            leftButton = new POVButton(hid, 270, povNumber);
        }

        return leftButton;
    }

    public POVButton upLeft() {
        if (upLeftButton == null) {
            upLeftButton = new POVButton(hid, 315, povNumber);
        }

        return upLeftButton;
    }
}
