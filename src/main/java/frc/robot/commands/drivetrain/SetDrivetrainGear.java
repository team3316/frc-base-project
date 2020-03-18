package frc.robot.commands.drivetrain;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.Gear;

/**
 * SetDrivetrainGear
 */
public class SetDrivetrainGear extends DBugCommand {
    private Gear _wantedState;

    public SetDrivetrainGear(Gear wantedState) {
        this._wantedState = wantedState;
    }

    @Override
    public void init() {
        System.out.println("Setting gear to " + this._wantedState.toString());
        Robot.drivetrain.setGear(this._wantedState);
    }

    @Override
    public void execute() {
        // Nothing
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    protected void fin(boolean interrupted) {
        // Nothing
    }
    
}