package frc.robot.commands.drivetrain;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;

/**
 * SetDrivetrainGear
 */
public class HoldPosition extends DBugCommand {
    public HoldPosition() {
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void init() {
        Robot.drivetrain.setPercent(0, 0);
    }

    @Override
    public void execute() {
        // Nothing
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    protected void fin(boolean interrupted) {
        // Nothing
    }
}