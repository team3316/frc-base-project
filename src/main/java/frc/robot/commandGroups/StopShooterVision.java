package frc.robot.commandGroups;

import com.team3316.kit.commands.DBugCommandGroup;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drivetrain.SetDrivetrainGear;
import frc.robot.subsystems.Drivetrain.Gear;

/**
 * StopShooterVision
 */
public class StopShooterVision extends DBugCommandGroup {
    public StopShooterVision() {
        // SetDrivetrainGear is wrapped in an InstantCommand because
        // in order for us to interrupt the HoldPosition command we need
        // a command that requires the drivetrain subsystem to run
        add(() -> new InstantCommand(new Runnable() {
                @Override
                public void run() {
                    new SetDrivetrainGear(Gear.HIGH).schedule();
                }
            }, Robot.drivetrain),
            () -> new StopShooting());
    }
}