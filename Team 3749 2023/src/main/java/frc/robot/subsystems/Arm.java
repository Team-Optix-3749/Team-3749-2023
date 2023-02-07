// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class Arm extends SubsystemBase {
    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;

    public Arm() {
        shoulderLeft = new CANSparkMax(16, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(15, MotorType.kBrushless);

        shoulderLeft.setIdleMode(IdleMode.kCoast);
        shoulderRight.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic() {
    }
}
