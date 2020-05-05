package frc.robot.autonomous.trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.autonomous.TrajectoryGeneration;

/**
 * ExampleTrajectory
 */
public class ExampleTrajectory extends TrajectoryGeneration {
    public static String fileName = "TestTrajectory.json";
    public ExampleTrajectory() {
        super(
            fileName,
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
            List.of(
                new Translation2d(0.5, 0.0)//,
                // new Translation2d(2.0, 1.0)
            ),
            new Pose2d(1.0, 0.0, new Rotation2d(Math.toRadians(90)))
        );
    }

    //// Uncomment to run ////
    // public static void main(String... args) {
    //     new TestTrajectory();
    // }
}