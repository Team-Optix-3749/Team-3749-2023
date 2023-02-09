package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawAuto extends SubsystemBase {
    public void getcoords() {
  
      // 2018 cross scale auto waypoints.
      var sideStart = new Pose2d(6,7,
          Rotation2d.fromDegrees(-180));
      var crossScale = new Pose2d(, Units.feetToMeters(6.8),
          Rotation2d.fromDegrees(-160));
  
      var interiorWaypoints = new ArrayList<Translation2d>();
      interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
      interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));
  
      TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
      config.setReversed(true);
  
      var trajectory = TrajectoryGenerator.generateTrajectory(
          sideStart,
          interiorWaypoints,
          crossScale,
          config);
    }
  }
  
