package frc.robot.commandGroups;

import com.team3316.kit.commands.DBugCommandGroup;

import frc.robot.Robot;
import frc.robot.commands.extractor.SetExtractorState;
import frc.robot.commands.pizza.SetPizzaState;
import frc.robot.commands.shooter.NewSetShooterState;
import frc.robot.subsystems.Extractor.ExtractorState;
import frc.robot.subsystems.Pizza.PizzaState;
import frc.robot.subsystems.Shooter.ShooterState;

/**
 * StopShooting
 */
public class StopShooting extends DBugCommandGroup {
    public StopShooting() {
        add(() -> new SetExtractorState(ExtractorState.NONE),
            () -> new NewSetShooterState(ShooterState.NONE),
            () -> new SetPizzaState(PizzaState.NONE));
    }
}