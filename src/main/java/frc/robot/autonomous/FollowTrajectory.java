package frc.robot.autonomous;

import java.nio.file.Path;
import com.team3316.kit.commands.DBugCommand;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Utils;

/**
 * FollowTrajectory
 * Defines and executes a Ramsete controller that follows a pregenerated JSON trajectory
 */
public class FollowTrajectory extends DBugCommand {
  private RamseteCommand _ramseteCommand;
  private AutoBrakeMode _brakeMode;

  public FollowTrajectory(String fileName, AutoBrakeMode brakeMode) {
    addRequirements(Robot.drivetrain);

    // Handle braking requirements
    if (brakeMode != AutoBrakeMode.NONE) Robot.drivetrain.setBrake(true);

    Path trajectoryPath = Constants.Autonomous.Directories.robotTrajectoriesDir.resolve(fileName);
    Trajectory trajectory = Utils.deserializeTrajectory(trajectoryPath);

    this._ramseteCommand = new RamseteCommand(
      trajectory,
      Robot.drivetrain::getPose,
      new RamseteController(Constants.Autonomous.kRamseteB, Constants.Autonomous.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.Autonomous.ksVolts,
        Constants.Autonomous.kvVoltSecondsPerMeter,
        Constants.Autonomous.kaVoltSecondsSquaredPerMeter),
      Robot.drivetrain.DriveKinematics,
      Robot.drivetrain::getWheelSpeeds,
      new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
      new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      Robot.drivetrain::tankDriveVolts,
      Robot.drivetrain
    );

    this._brakeMode = brakeMode;
  }

  @Override
  public void init() {
    // Start the ramsete command
    this._ramseteCommand.schedule();
  }

  @Override
  public void execute() {
    Robot.drivetrain.updateOdometry();
  }

  @Override
  public boolean isFinished() {
    return _ramseteCommand.isFinished();
  }

  @Override
  protected void fin(boolean interrupted) {
    new InstantCommand(() -> {
      if (this._brakeMode == AutoBrakeMode.WHILE_RUNNING_DELAY) {
        Robot.drivetrain.tankDriveVolts(0, 0);
        new TemporaryBrake(Constants.Autonomous.brakeDelay).schedule();
      } else if (this._brakeMode == AutoBrakeMode.WHILE_RUNNING_INSTANT) Robot.drivetrain.setBrake(false);
    }, Robot.drivetrain);
  }
}