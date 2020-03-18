package frc.robot.commands.pizza;

import com.team3316.kit.commands.DBugCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Pizza.PizzaState;

public class SetPizzaState extends DBugCommand {
	PizzaState _wantedState;

	public SetPizzaState(PizzaState wanted) {
		addRequirements(Robot.pizza);
		this._wantedState = wanted;
	}

	@Override
	public void init() {
		Robot.pizza.setPizzaState(this._wantedState);
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