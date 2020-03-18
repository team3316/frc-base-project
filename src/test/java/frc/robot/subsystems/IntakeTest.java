package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.Intake.RollerState;

/**
 * IntakeTest
 */
public class IntakeTest {

    @BeforeAll
    public static void init() {
        Robot.intake = new Intake();
    }

    @Test
    public void rollerMovementTest() {
        Robot.intake.setPosition(IntakePosition.IN);
        assertEquals(Robot.intake.getPosition(), IntakePosition.IN);
    }

    @Test
    public void rollerMovementVelocityTest() {
        Robot.intake.setPosition(IntakePosition.IN);
        assertEquals(Robot.intake.getPosition().getValue(), Constants.Intake.IntakePosition.IN.percent, 0.1);
    }

    @Test
    public void rollerStateTest() {
        Robot.intake.setRollerState(RollerState.IN);
        assertEquals(Robot.intake.getRollerState(), RollerState.IN);
    }

    @Test
    public void rollerStateTestPercent() {
        Robot.intake.setRollerState(RollerState.IN);
        assertEquals(Robot.intake.getRollerState().getValue(), Constants.Intake.Roller.RollerState.IN.voltage, 0.1);
    }
}