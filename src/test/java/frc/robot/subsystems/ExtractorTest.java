package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Extractor.ExtractorState;;

/**
 * PizzaTest
 */
public class ExtractorTest {

    @BeforeAll
    public static void init() {
        Robot.extractor = new Extractor();
    }

    @Test
    public void setRollerState() {
        Robot.extractor.setExtractorState(ExtractorState.OUT);
        assertEquals(Robot.extractor.getExtractorState(), ExtractorState.OUT);
    }
}