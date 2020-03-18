package frc.robot.commands.pizza;

import static org.junit.Assert.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.Robot;
import frc.robot.subsystems.Pizza;
import frc.robot.subsystems.Pizza.PizzaState;

/**
 * PizzaCommandsTests
 */
public class PizzaCommandsTests {

    @BeforeAll
    public static void init() {
        Robot.pizza = new Pizza();
    }
    
    @Test
    public void cmdPizzaStateTestInit() {
        new SetPizzaState(PizzaState.CLOCKWISE).init();

        assertEquals(Robot.pizza.getPizzaState(), PizzaState.CLOCKWISE);
    }
}