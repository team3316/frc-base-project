package frc.robot.commands.drivetrain;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;

/**
 * SetDrivetrainDistance
 */
public class SetDrivetrainDistance extends DBugCommand {

    private double _dist;

    public SetDrivetrainDistance(double dist) {
        addRequirements(Robot.drivetrain);
        this._dist = dist;
    }

    @Override
    public void init() {
        Robot.drivetrain.setDistance(this._dist, this._dist);
    }

    @Override
    public void execute() {
        // Nothing
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    protected void fin(boolean interrupted) {
        // TODO Auto-generated method stub
    }    
}