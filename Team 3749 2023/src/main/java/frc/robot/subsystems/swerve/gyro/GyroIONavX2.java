// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

/** IO implementation for Pigeon2 */
public class GyroIONavX2 implements GyroIO {
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public GyroIONavX2() {

  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawAngle = -gyro.getYaw();
    inputs.pitchAngle = gyro.getPitch();

  }

  public void calibrate(){
    gyro.calibrate();
  }
  public void reset() {
    gyro.reset();
  }

}