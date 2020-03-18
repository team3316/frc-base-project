package frc.robot.commands.PCPoosher;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;
import frc.robot.subsystems.PCPoosher.PCPoosherState;

/**
 * SetPCPoosherState
 */
public class SetPCPoosherState extends DBugCommand {

    private PCPoosherState _wantedState;

    public SetPCPoosherState(PCPoosherState wantedState) {
        addRequirements(Robot.pcPoosher);
        this._wantedState = wantedState;
    }

    @Override
    public void init() {
        Robot.pcPoosher.setState(this._wantedState);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isFinished() {
        return Robot.pcPoosher.getState() == this._wantedState;
    }

    @Override
    protected void fin(boolean interrupted) {
        // TODO Auto-generated method stub
    }

}