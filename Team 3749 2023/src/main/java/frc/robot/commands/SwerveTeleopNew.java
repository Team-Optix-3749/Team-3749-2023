// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Rohin Sood
 * 
 *         Controlling the
 */
public class SwerveTeleopNew extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final DrivetrainNew swerveSubsystem;
  private final DoubleSupplier xSpdFunction, ySpdFunction, turningSpdFunction;
  private final BooleanSupplier fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  // Initializes the BaseCommand
  public SwerveTeleopNew(DrivetrainNew swerveSubsystem,
      DoubleSupplier xSpdFunction, DoubleSupplier ySpdFunction, DoubleSupplier turningSpdFunction,
      BooleanSupplier fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(Constants.DrivetrainOld.tele_drive_max_acceleration_units_per_second);
    this.yLimiter = new SlewRateLimiter(Constants.DrivetrainOld.tele_drive_max_acceleration_units_per_second);
    this.turningLimiter = new SlewRateLimiter(
        Constants.DrivetrainOld.tele_drive_max_angular_acceleration_units_per_second);
    addRequirements(swerveSubsystem);
  }

  // Run on command init
  @Override
  public void initialize() {
  }

  // Run every 20 ms
  @Override
  public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.getAsDouble();
        double ySpeed = ySpdFunction.getAsDouble();
        double turningSpeed = turningSpdFunction.getAsDouble();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.DrivetrainOld.deadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.DrivetrainOld.deadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.DrivetrainOld.deadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother by limiting acceleration
        xSpeed = xLimiter.calculate(xSpeed) * Constants.DrivetrainOld.tele_drive_max_speed_meters_per_second;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DrivetrainOld.tele_drive_max_speed_meters_per_second;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.DrivetrainOld.tele_drive_max_angular_acceleration_units_per_second;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.getAsBoolean()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        }
        
        else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.DrivetrainOld.driveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
  }

  // Run on command finish
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return false;
  }
}