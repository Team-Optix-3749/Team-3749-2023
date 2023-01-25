// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing;
import frc.robot.utils.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Noah Simon
 * @summary
 *          Controlling the Test Drivetrain subsystem through use of joysticks,
 *          drive and turning motors
 */
public class TurnTo0Degrees extends CommandBase {
        @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

        private final DrivetrainTesting swerveSubsystem;

        // Initializes the BaseCommand
        public TurnTo0Degrees(DrivetrainTesting swerveSubsystem) {
            this.swerveSubsystem = swerveSubsystem;
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
            // 4. Send processed inputs to the drive() function
            swerveSubsystem.TurnToDegrees(Math.PI / 2);
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