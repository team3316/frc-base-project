package frc.robot.commandGroups;

import com.team3316.kit.commands.DBugCommandGroup;

import frc.robot.Robot;
import frc.robot.commands.drivetrain.GoToYaw;
import frc.robot.commands.drivetrain.HoldPosition;
import frc.robot.commands.drivetrain.SetDrivetrainGear;
import frc.robot.subsystems.Drivetrain.Gear;

/**
 * StartShooterVision
 */
public class StartShooterVision extends DBugCommandGroup {
    public StartShooterVision() {
        add(() -> new SetDrivetrainGear(Gear.LOW));
        add(() -> new GoToYaw());
        add(() -> new HoldPosition(),
            () -> new StartShooterUpper());
    }
}