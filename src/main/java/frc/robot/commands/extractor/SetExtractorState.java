package frc.robot.commands.extractor;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Extractor.ExtractorState;

/**
 * SetExtractorsState
 */
public class SetExtractorState extends DBugCommand {

    ExtractorState _wantedRollerState;

    public SetExtractorState(ExtractorState wanted) {
        addRequirements(Robot.extractor);
        this._wantedRollerState = wanted;
    }

    @Override
    public void init() {
        Robot.extractor.setExtractorState(this._wantedRollerState);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    protected void fin(boolean interrupted) {
        // Nothing
    }

    
}