// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.


package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
public interface SwerveModule {
  public static class ModuleData {
    public SwerveModulePosition position = new SwerveModulePosition();
    public SwerveModuleState theoreticalState = new SwerveModuleState();
    public double drivePositionM = 0.0;
    public double driveVelocityMPerSec = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTempCelcius = 0.0;

    public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnTempCelcius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateData(ModuleData data) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDesiredState(SwerveModuleState state) {}


  /** Run the turn motor at the specified voltage. */
  public default void stop() {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}
}