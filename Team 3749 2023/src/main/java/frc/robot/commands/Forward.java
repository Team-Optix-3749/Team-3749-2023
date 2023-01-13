package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainNew;

public class Forward extends CommandBase {
    private final DrivetrainNew swerveSubsystem;
    

    public Forward(DrivetrainNew swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 1.0, 0)); 

    }
}
