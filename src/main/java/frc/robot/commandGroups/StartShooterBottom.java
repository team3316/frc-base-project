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
 * StartShooterBottom
 */
public class StartShooterBottom extends DBugCommandGroup {
    public StartShooterBottom() {
        add(() -> new NewSetShooterState(ShooterState.SHOOT_BOTTOM),
            () -> new SetPizzaState(PizzaState.CLOCKWISE_BOTTOM),
            () -> new SetExtractorState(ExtractorState.OUT));
    }
}