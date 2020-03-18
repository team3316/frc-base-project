package frc.robot.commands.intake;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakePosition;

/**
 * SetIntakeState
 */
public class SetIntakeState extends DBugCommand {
    IntakePosition _wantedState;

    public SetIntakeState(IntakePosition wanted) {
        this._wantedState = wanted;
    }

    @Override
    public void init() {
        Robot.intake.setPosition(this._wantedState);
    }

    @Override
    public void execute() {
        // Nothing
    }

    @Override
    public boolean isFinished() {
        return Robot.intake.getPosition() == this._wantedState;
    }

    @Override
    protected void fin(boolean interrupted) {
        Robot.intake.setPositionKeep(this._wantedState);
    }
}