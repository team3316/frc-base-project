package frc.robot.commandGroups;

import com.team3316.kit.commands.DBugCommandGroup;

import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.intake.SetRollersState;
import frc.robot.commands.pizza.SetPizzaState;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.Intake.RollerState;
import frc.robot.subsystems.Pizza.PizzaState;

/**
 * StartCollection
 */
public class StartCollection extends DBugCommandGroup {
    public StartCollection() {
        add(() -> new SetIntakeState(IntakePosition.OUT),
            () -> new SetPizzaState(PizzaState.CLOCKWISE),
            () -> new SetRollersState(RollerState.IN));
    }
}