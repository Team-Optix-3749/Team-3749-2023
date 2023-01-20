// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing;

import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.SwerveENUMS;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Rohin Sood
 * 
 *         Controlling the DrivetrainNew subsystem through use of joysticks
 */
public class MoveIndividualTurning extends CommandBase {
        @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

        private final DrivetrainTesting swerveSubsystem;
        private final DoubleSupplier xSpdFunction, ySpdFunction, turningSpdFunction;
        private final BooleanSupplier fieldOrientedFunction;
        private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
        private final SwerveENUMS moduleENUM;

        // Initializes the BaseCommand
        public MoveIndividualTurning(DrivetrainTesting swerveSubsystem,
                        DoubleSupplier xSpdFunction, DoubleSupplier ySpdFunction, DoubleSupplier turningSpdFunction,
                        BooleanSupplier fieldOrientedFunction, SwerveENUMS moduleENUM) {
                this.swerveSubsystem = swerveSubsystem;
                this.xSpdFunction = xSpdFunction;
                this.ySpdFunction = ySpdFunction;
                this.turningSpdFunction = turningSpdFunction;
                this.fieldOrientedFunction = fieldOrientedFunction;
                this.xLimiter = new SlewRateLimiter(Constants.DrivetrainNew.max_speed);
                this.yLimiter = new SlewRateLimiter(Constants.DrivetrainNew.max_speed);
                this.turningLimiter = new SlewRateLimiter(
                                Constants.DrivetrainNew.max_angular_speed);
                this.moduleENUM = moduleENUM;
                addRequirements(swerveSubsystem);
        }

        // Run on command init
        @Override
        public void initialize() {
        }

        // Run every 20 ms
        // TESTING CHANGES: Multiply speeds by 0.1 BEFORE limiter. Smart dashboard
        @Override
        public void execute() {
                final var xSpeed = -xLimiter.calculate(MathUtil.applyDeadband(xSpdFunction.getAsDouble(), 0.02))
                                * Constants.DrivetrainNew.max_speed;

                // Get the y speed or sideways/strafe speed. We are inverting this because
                // we want a positive value when we pull to the left. Xbox controllers
                // return positive values when you pull to the right by default.
                final var ySpeed = -yLimiter.calculate(MathUtil.applyDeadband(ySpdFunction.getAsDouble(), 0.02))
                                * Constants.DrivetrainNew.max_speed;

                // Get the rate of angular rotation. We are inverting this because we want a
                // positive value when we pull to the left (remember, CCW is positive in
                // mathematics). Xbox controllers return positive values when you pull to
                // the right by default.
                final var rot = -turningLimiter
                                .calculate(MathUtil.applyDeadband(turningSpdFunction.getAsDouble(), 0.02))
                                * Constants.DrivetrainNew.max_angular_speed;

                SmartDashboard.putNumber("xSpeed", xSpeed);
                SmartDashboard.putNumber("ySpeed", ySpeed);
                SmartDashboard.putNumber("rotationSpeed", rot);

                // 4. Send processed inputs to the drive() function
                swerveSubsystem.moveIndividualModule(xSpeed, ySpeed, rot, false, moduleENUM, false, true);
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