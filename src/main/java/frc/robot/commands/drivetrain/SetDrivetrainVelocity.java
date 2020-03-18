package frc.robot.commands.drivetrain;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;

/**
 * SetDrivetrainVelocity
 */
public class SetDrivetrainVelocity extends DBugCommand {

    private double _velocity;

    public SetDrivetrainVelocity(double vel) {
        this._velocity = vel;
    }

    @Override
    public void init() {
        Robot.drivetrain.setVelocity(this._velocity);
    }

    @Override
    public void execute() {

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