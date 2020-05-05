package frc.robot.autonomous;

import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants;
import frc.robot.utils.Utils;

/**
 * TrajectoryGeneration
 */
public class TrajectoryGeneration {
  /**
   * Generate Trajectory and call serializeTrajectory to write it to file as JSON
   * @param fileName name of the trajectory file (i.e. XXXXXX.json)
   * @param start start point with heading
   * @param interiorWaypoints list of interior point, heading calculated automaticly 
   * @param end end point with heading
   */
  public TrajectoryGeneration(String fileName, Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end) {
    DifferentialDriveKinematics DriveKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.wheelDistance);

    // Create a voltage constraint to ensure we don't accelerate too fast
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.Autonomous.ksVolts,
          Constants.Autonomous.kvVoltSecondsPerMeter,
          Constants.Autonomous.kaVoltSecondsSquaredPerMeter),
          DriveKinematics, 10);
          
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.Autonomous.kMaxSpeedMetersPerSecond,
      Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    
    Path trajectoryPath = Constants.Autonomous.Directories.localTrajectoriesDir.resolve(fileName);
    Utils.serializeTrajectory(trajectoryPath, trajectory);
  }
}