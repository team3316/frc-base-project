package frc.robot.commands.intake;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Intake.RollerState;

/**
 * SetRollersState
 */
public class SetRollersState extends DBugCommand {

    RollerState _wantedState;

    public SetRollersState(RollerState wanted) {
        this._wantedState = wanted;
    }

    @Override
    public void init() {
        Robot.intake.setRollerState(this._wantedState);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    protected void fin(boolean interrupted) {
        Robot.intake.setRollerState(RollerState.DEFAULT);
    }    
}