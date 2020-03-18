package frc.robot.commandGroups;

import com.team3316.kit.commands.DBugCommandGroup;

import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.intake.SetRollersState;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.Intake.RollerState;

/**
 * LoadingBayCollection
 */
public class LoadingBayCollection extends DBugCommandGroup {

    public LoadingBayCollection() {
        add(() -> new SetRollersState(RollerState.OUT), () -> new SetIntakeState(IntakePosition.IN));
    }
}