package frc.robot.commandGroups;

import com.team3316.kit.commands.DBugCommandGroup;

import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.intake.SetRollersState;
import frc.robot.commands.pizza.SetPizzaState;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.Intake.RollerState;
import frc.robot.subsystems.Pizza.PizzaState;

/**
 * StopCollection
 */
public class StopCollection extends DBugCommandGroup {
    public StopCollection() {
        add(() -> new SetIntakeState(IntakePosition.IN));
        add(() -> new SetPizzaState(PizzaState.NONE),
            () -> new SetRollersState(RollerState.DEFAULT));
    }
}