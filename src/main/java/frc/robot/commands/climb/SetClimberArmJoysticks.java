package frc.robot.commands.climb;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;

/**
 * SetClimberArmJoysticks
 */
public class SetClimberArmJoysticks extends DBugCommand {

    public SetClimberArmJoysticks() {
        addRequirements(Robot.climber);
    }

    @Override
    public void init() {
        // Nothing
    }
    
    @Override
    public void execute() {
        // System.out.println(Robot.joysticks.getLeftXboxY());
        Robot.climber.setElevatorPercentage(Robot.joysticks.getLeftXboxY());
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