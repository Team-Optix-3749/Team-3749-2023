// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
    private final GenericHID m_hid;
    private final int m_povNumber;

    private POVButton m_upButton; // 0 degrees
    private POVButton m_upRightButton; // 45 degrees
    private POVButton m_rightButton; // 90 degrees
    private POVButton m_downRightButton; // 135 degrees
    private POVButton m_downButton; // 180 degrees
    private POVButton m_downLeftButton; // 225 degrees
    private POVButton m_leftButton; // 270 degrees
    private POVButton m_upLeftButton; // 315 degrees

    public POV(GenericHID hid) {
        this(hid, 0);
    }

    public POV(GenericHID hid, int povNumber) {
        m_hid = hid;
        m_povNumber = povNumber;
    }

    public POVButton up() {
        if (m_upButton == null) {
            m_upButton = new POVButton(m_hid, 0, m_povNumber);
        }

        return m_upButton;
    }

    public POVButton upRight() {
        if (m_upRightButton == null) {
            m_upRightButton = new POVButton(m_hid, 45, m_povNumber);
        }

        return m_upRightButton;
    }

    public POVButton right() {
        if (m_rightButton == null) {
            m_rightButton = new POVButton(m_hid, 90, m_povNumber);
        }

        return m_rightButton;
    }

    public POVButton downRight() {
        if (m_downRightButton == null) {
            m_downRightButton = new POVButton(m_hid, 135, m_povNumber);
        }

        return m_downRightButton;
    }

    public POVButton down() {
        if (m_downButton == null) {
            m_downButton = new POVButton(m_hid, 180, m_povNumber);
        }

        return m_downButton;
    }

    public POVButton downLeft() {
        if (m_downLeftButton == null) {
            m_downLeftButton = new POVButton(m_hid, 225, m_povNumber);
        }

        return m_downLeftButton;
    }

    public POVButton left() {
        if (m_leftButton == null) {
            m_leftButton = new POVButton(m_hid, 270, m_povNumber);
        }

        return m_leftButton;
    }

    public POVButton upLeft() {
        if (m_upLeftButton == null) {
            m_upLeftButton = new POVButton(m_hid, 315, m_povNumber);
        }

        return m_upLeftButton;
    }
}
