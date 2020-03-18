package frc.robot.commands.general;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;

/**
 * EnableCompressor
 */
public class EnableCompressor extends DBugCommand {

    @Override
    public void init() {
        Robot.compressor.start();
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