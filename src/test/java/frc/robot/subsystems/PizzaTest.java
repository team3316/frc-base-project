package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.Robot;
import frc.robot.subsystems.Pizza.PizzaState;

/**
 * PizzaTest
 */
public class PizzaTest {

    @BeforeAll
    public static void init() {
        Robot.pizza = new Pizza();
    }
    
    @Test
    public void setPizzaState() {
        Robot.pizza.setPizzaState(PizzaState.CLOCKWISE);
        assertEquals(Robot.pizza.getPizzaState(), PizzaState.CLOCKWISE);
    }

}