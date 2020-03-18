package frc.robot.commands.general;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;

/**
 * DisableCompressor
 */
public class DisableCompressor extends DBugCommand {

    @Override
    public void init() {
        Robot.compressor.stop();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    protected void fin(boolean interrupted) {

    }

}