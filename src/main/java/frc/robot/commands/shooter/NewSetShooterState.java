package frc.robot.commands.shooter;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Shooter.ShooterState;

/**
 * NewSetShooterState
 */
public class NewSetShooterState extends DBugCommand {
    private ShooterState _state;

    public NewSetShooterState(ShooterState state){
        addRequirements(Robot.shooter);

        this._state = state;
    }

    @Override
    public void init() {
        Robot.shooter.setPeriodicShooterState(this._state);
    }

    @Override
    public void execute() {
        // Nothing
    }

    @Override
    public boolean isFinished() {
        return Robot.shooter.getShooterState() == this._state;
    }

    @Override
    protected void fin(boolean interrupted) {
        // Nothing
    }
}