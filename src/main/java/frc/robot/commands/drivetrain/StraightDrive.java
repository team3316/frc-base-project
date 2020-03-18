package frc.robot.commands.drivetrain;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Constants;
import frc.robot.Robot;

/**
 * SetDrivetrainVelocity
 */
public class StraightDrive extends DBugCommand {

    public StraightDrive() {
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void init() {
        // Nothing
    }

    @Override
    public void execute() {
        // Nothing
        Robot.drivetrain.setVelocity(Robot.joysticks.getRightY() * Constants.Drivetrain.straightDriveMaxVelocityInRawUnits, Robot.joysticks.getRightY() * Constants.Drivetrain.straightDriveMaxVelocityInRawUnits);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    protected void fin(boolean interrupted) {
        Robot.drivetrain.setPercent(0.0);
    }    
}