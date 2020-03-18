package frc.robot.commands.drivetrain;

import com.team3316.kit.commands.DBugCommand;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * SetDrivetrainTime
 */
public class SetDrivetrainTime extends DBugCommand {

    private Timer _timer = new Timer();
    private double _seconds, _percentage;

    public SetDrivetrainTime(double seconds, double percentage) {
        this._seconds = seconds;
        this._percentage = percentage;
    }

    @Override
    public void init() {
        // Timer
        this._timer = new Timer();
        this._timer.reset();
        this._timer.start();

        // Set drivetrain
        Robot.drivetrain.setPercent(this._percentage);
    }

    @Override
    public void execute() {
        // Nothing
    }

    @Override
    public boolean isFinished() {
        System.out.println(this._timer.get() + " > " + this._seconds);
        return this._timer.get() > this._seconds;
    }

    @Override
    protected void fin(boolean interrupted) {
        Robot.drivetrain.setPercent(0);
    }
  
}