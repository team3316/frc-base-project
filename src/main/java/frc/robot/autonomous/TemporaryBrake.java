package frc.robot.commands.autonomous;

import com.team3316.kit.commands.DBugCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * TemporaryBrake
 * Sets mode to brake for a limited amount of time and then to coast. 
 */
public class TemporaryBrake extends DBugCommand {
  private double _delay;
  private Timer _timer;

  public TemporaryBrake(double delay) {
    addRequirements(Robot.drivetrain);

    this._delay = delay;
    this._timer = new Timer();
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void init() {
    Robot.drivetrain.setBrake(true);
    this._timer.reset();
    this._timer.start();
  }

  @Override
  public void execute() {
      // Nothing
  }

  @Override
  public boolean isFinished() {
      return this._timer.get() > this._delay;
  }

  @Override
  protected void fin(boolean interrupted) {
    Robot.drivetrain.setBrake(false);
    this._timer.stop();
  }
}