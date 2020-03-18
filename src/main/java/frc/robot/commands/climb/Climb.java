package frc.robot.commands.climb;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Climber.ClimberState;

/**
 * Climb
 */
public class Climb extends DBugCommand {
    public Climb() {
        addRequirements(Robot.climber);
    }

    @Override
    public void init() {
        Robot.climber.setClimberState(ClimberState.PULL);
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
        Robot.climber.setClimberState(ClimberState.NONE);
    }
}