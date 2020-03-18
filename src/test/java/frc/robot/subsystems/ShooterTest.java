package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.Robot;
import frc.robot.subsystems.Shooter.ShooterState;

/**
 * ShooterTest
 */
public class ShooterTest {

    @BeforeAll
    public static void init() {
        Robot.shooter = new Shooter();
    }
    
    @Test
    public void testSetStateShoot() {
        Robot.shooter.setShooterState(ShooterState.SHOOT);

        assertEquals(ShooterState.SHOOT , Robot.shooter.getShooterState());
    }

    @Test
    public void testSetStateNone() {
        Robot.shooter.setShooterState(ShooterState.NONE);

        assertEquals(ShooterState.NONE , Robot.shooter.getShooterState());
    }
}