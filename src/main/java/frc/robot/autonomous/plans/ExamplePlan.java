package frc.robot.autonomous.plans;

import com.team3316.kit.commands.DBugCommandGroup;

import frc.robot.autonomous.AutoBrakeMode;
import frc.robot.autonomous.FollowTrajectory;
import frc.robot.autonomous.trajectories.ExampleTrajectory;

/**
 * ExamplePlan
 */
public class ExamplePlan extends DBugCommandGroup {
  public ExamplePlan() {
    add(() -> new FollowTrajectory(ExampleTrajectory.fileName, AutoBrakeMode.ALWAYS));
  }
}