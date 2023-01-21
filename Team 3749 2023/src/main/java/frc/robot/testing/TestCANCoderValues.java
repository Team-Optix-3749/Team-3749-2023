// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing;
import edu.wpi.first.wpilibj2.command.CommandBase;

/***
 * @author Noah Simon
 * @summary
 *          Controlling the Test Drivetrain subsystem through use of joysticks,
 *          drive and turning motors
 */
public class TestCANCoderValues extends CommandBase {
        @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

        DrivetrainTesting swerveSubsystem;

        // Initializes the BaseCommand
        public TestCANCoderValues(DrivetrainTesting swerveSubsystem) {
                this.swerveSubsystem=swerveSubsystem;
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
               swerveSubsystem.logAbsoluteEncoderValues();

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