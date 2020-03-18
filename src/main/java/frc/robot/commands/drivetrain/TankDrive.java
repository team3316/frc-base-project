package frc.robot.commands.drivetrain;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;

/**
 * TankDrive
 */
public class TankDrive extends DBugCommand {

    public TankDrive() {
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub

    }

    @Override
    public void execute() {
        Robot.drivetrain.setPercent(Robot.joysticks.getLeftY(), Robot.joysticks.getRightY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    protected void fin(boolean interrupted) {
        // Nothing
        Robot.drivetrain.setPercent(0.0);
    }
    
}