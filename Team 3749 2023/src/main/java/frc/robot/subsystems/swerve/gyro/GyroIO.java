// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public double yawAngle = 0.0;
    public double pitchAngle = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
  public default void calibrate() {}
  public default void reset() {}


}