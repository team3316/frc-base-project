package frc.robot.commands.drivetrain;

import com.team3316.kit.control.PIDFGains;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.Gear;

/**
 * GoToYaw //excel trendline approximation: = 0.0721* Math.pow(x, 0.4042)
 */
public class GoToYaw extends CommandBase {
    double currentYaw, visionYaw, _gyroOffset;
    double output;

    PIDController _controller;

    double visionTolerance;

    double TURN_TOLERANCE = 2;
    double SECOND_LOOP_TURN_TOLERANCE = 1;
    double TURN_VELOCITY_TOLERANCE = 1;
    double TURN_I_RANGE = 2;

    public GoToYaw() {
        addRequirements(Robot.drivetrain);

        this.visionYaw = 0;
        this.currentYaw = 0;
        this.output = 0;

        // Slow is fast actually.
        PIDFGains gainsTurn = Constants.Drivetrain.MotorControllers.TalonL.gainsTurn;
        _controller = new PIDController(gainsTurn.getP(), gainsTurn.getI(), gainsTurn.getD());
        _controller.setTolerance(TURN_TOLERANCE, TURN_VELOCITY_TOLERANCE);
        _controller.setSetpoint(0);
        _controller.setIntegratorRange(-TURN_I_RANGE, TURN_I_RANGE);
    }

    @Override
    public void initialize() {
        Robot.drivetrain.setBrake(true);
        Robot.drivetrain.setGear(Gear.LOW);
        resetControllers();
    }

    private void resetControllers() {
        _controller.reset();
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) { }
        this.visionYaw = Robot.visionServer.getYawAngle() + SmartDashboard.getNumber("YawOffset", 0.0);
        SmartDashboard.putNumber("visionYaw", this.visionYaw);
        this._gyroOffset = Robot.drivetrain.getYawDegrees() + this.visionYaw;
    }

    @Override
    public void execute() {
        this.currentYaw = this._gyroOffset - Robot.drivetrain.getYawDegrees();
        
        this.output = this._controller.calculate(this.currentYaw);
        
        SmartDashboard.putNumber("error", this.currentYaw);

        Robot.drivetrain.setPercent(-output, output);
    }

    @Override
    public boolean isFinished() {
        if (this._controller.atSetpoint()) {
            resetControllers();
            this._controller.setTolerance(SECOND_LOOP_TURN_TOLERANCE, TURN_VELOCITY_TOLERANCE);
            if (Math.abs(this.visionYaw) < SECOND_LOOP_TURN_TOLERANCE) {
                return true;
            }
        }
        return false;
    }

    @Override
    protected void finalize() throws Throwable { }

     }
    