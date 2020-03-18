package frc.robot.commands.autonomous.plans;

import com.team3316.kit.commands.DBugCommandGroup;

import frc.robot.commands.drivetrain.SetDrivetrainTime;

/**
 * StraightLinePlan
 */
public class StraightLinePlan extends DBugCommandGroup {
  public StraightLinePlan() {
    add(() -> new SetDrivetrainTime(2.0, 0.4));
  }
}