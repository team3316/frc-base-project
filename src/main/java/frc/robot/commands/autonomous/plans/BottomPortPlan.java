package frc.robot.commands.autonomous.plans;

import com.team3316.kit.commands.DBugCommandGroup;

import frc.robot.commandGroups.StartShooterBottom;
import frc.robot.commandGroups.StopShooting;
import frc.robot.commands.drivetrain.SetDrivetrainTime;

/**
 * BottomPortPlan
 */
public class BottomPortPlan extends DBugCommandGroup {
  public BottomPortPlan() {
    add(() -> new SetDrivetrainTime(3.0, 0.4));
    add(() -> new StartShooterBottom());
    add(() -> new StopShooting());
  }
}