package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team3316.kit.DBugSubsystem;
import com.team3316.kit.motors.DBugSparkMax;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.shooter.NewSetShooterState;
import frc.robot.utils.Utils;

public class Shooter extends DBugSubsystem {
    private DBugSparkMax _sparkMaxLeft;
    private DBugSparkMax _sparkMaxRight;

    private double _lastSpeed;

    private ShooterState _wantedState;

    private boolean m_periodic = false;

    private class FlywheelController {
        private double kP, kF;
        private double setpoint;

        public FlywheelController(double kP, double kF) {
            this.kP = kP;
            this.kF = kF;

            this.setpoint = 0;
        }

        public void setSetpoint(double setpoint) {
            this.setpoint = setpoint;
        }

        public double calculate(double velocity) {
            return kP * (setpoint - velocity) + kF * setpoint;
        }
    }

    private FlywheelController leftController;
    private FlywheelController rightController;

    public Shooter() {

        this._lastSpeed = 0.0; 

        this._sparkMaxLeft = (DBugSparkMax) Utils.getBean("shooter.sparkMaxLeft");
        this._sparkMaxRight = (DBugSparkMax) Utils.getBean("shooter.sparkMaxRight");

        this._sparkMaxLeft.setInverted(Constants.Shooter.MotorControllers.SparkMaxLeft.inverted);
        this._sparkMaxRight.setInverted(Constants.Shooter.MotorControllers.SparkMaxRight.inverted);

        this._sparkMaxLeft.setSmartCurrentLimit(Constants.Shooter.peakCurrent);
        this._sparkMaxRight.setSmartCurrentLimit(Constants.Shooter.peakCurrent);

        this._sparkMaxLeft.setDistancePerRevolution(Constants.Shooter.MotorControllers.SparkMaxRight.dpr,
            Constants.Sensors.Encoders.SparkMAX.upr);
        this._sparkMaxRight.setDistancePerRevolution(Constants.Shooter.MotorControllers.SparkMaxRight.dpr,
            Constants.Sensors.Encoders.SparkMAX.upr);

        setGains();

        this._wantedState = ShooterState.NONE;
    }

    private void setVoltage(double leftVoltage, double rightVoltage) {
        _sparkMaxLeft.set(ControlMode.PercentOutput, leftVoltage / RobotController.getBatteryVoltage());
        _sparkMaxRight.set(ControlMode.PercentOutput, rightVoltage / RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {
        if (m_periodic) {
            setVoltage(leftController.calculate(_sparkMaxLeft.getVelocity()),
                    rightController.calculate(_sparkMaxRight.getVelocity()));
        }
    }

    public void setGains() {
        this._sparkMaxLeft.setupPIDF(Constants.Shooter.State.Shoot.gainsLeft.getP(),
            Constants.Shooter.State.Shoot.gainsLeft.getI(),
            Constants.Shooter.State.Shoot.gainsLeft.getD(),
            Constants.Shooter.State.Shoot.gainsLeft.getF());
        this._sparkMaxRight.setupPIDF(Constants.Shooter.State.Shoot.gainsRight.getP(),
            Constants.Shooter.State.Shoot.gainsRight.getI(),
            Constants.Shooter.State.Shoot.gainsRight.getD(),
            Constants.Shooter.State.Shoot.gainsRight.getF());
    }
    
    public double getDistanceLeft() {
        return this._sparkMaxLeft.getEncoder().getPosition();
    }

    public double getDistanceRight() {
        return this._sparkMaxLeft.getEncoder().getPosition();
    }

    /**
     * the state of the shooter.
     */
    public enum ShooterState {
        SHOOT(Constants.Shooter.State.Shoot.velocityLeft, Constants.Shooter.State.Shoot.velocityRight),
        SHOOT_BOTTOM(Constants.Shooter.State.Shoot.velocityLeft, Constants.Shooter.State.Shoot.velocityRight),
        NONE(Constants.Shooter.State.None.velocityLeft, Constants.Shooter.State.None.velocityRight);

        private double _velocityLeft, _velocityRight;

        private ShooterState(double velocityLeft, double velocityRight) {
            this._velocityLeft = velocityLeft;
            this._velocityRight = velocityRight;
        }

        public double getLeftVelocity() {
            return this._velocityLeft;
        }

        public double getRightVelocity() {
            return this._velocityRight;
        }
    }

    /**
     * set the state of the shooter.
     * @param state the shooter's state.
     */
    public void setShooterState(ShooterState state) {
        this._wantedState = state;
        this.m_periodic = false;

        if (state == ShooterState.SHOOT) {
            this._sparkMaxLeft.set(ControlMode.Velocity, state.getLeftVelocity());
            this._sparkMaxRight.set(ControlMode.Velocity, state.getRightVelocity());
        } else {
            this._sparkMaxLeft.set(ControlMode.PercentOutput, state.getLeftVelocity());
            this._sparkMaxRight.set(ControlMode.PercentOutput, state.getRightVelocity());
        }
    }

    public void setPeriodicShooterState(ShooterState state) {
        this._wantedState = state;
        this.m_periodic = true;

        // Numbers from robot characterization
        leftController = new FlywheelController(Constants.Shooter.State.Shoot.gainsLeft.getP(),
                Constants.Shooter.State.Shoot.gainsLeft.getF());
        rightController = new FlywheelController(Constants.Shooter.State.Shoot.gainsRight.getP(),
                Constants.Shooter.State.Shoot.gainsRight.getF());
        
        leftController.setSetpoint(state.getLeftVelocity());
        rightController.setSetpoint(state.getRightVelocity());
    }

    /**
     * return the current shooter state.
     * @return the current shooter state.
     */
    public ShooterState getShooterState() {
        
        if (
            Utils.isInNeighborhood(
                ShooterState.SHOOT.getLeftVelocity(),
                this._sparkMaxLeft.getVelocity(),
                Constants.Shooter.State.tolerance) 
        &&
            Utils.isInNeighborhood(
                ShooterState.SHOOT.getRightVelocity(),
                this._sparkMaxRight.getVelocity(),
                Constants.Shooter.State.tolerance)
        ) return ShooterState.SHOOT;

        if (
            Utils.isInNeighborhood(
                ShooterState.SHOOT_BOTTOM.getLeftVelocity(),
                this._sparkMaxLeft.getVelocity(),
                Constants.Shooter.State.tolerance) 
        &&
            Utils.isInNeighborhood(
                ShooterState.SHOOT_BOTTOM.getRightVelocity(),
                this._sparkMaxRight.getVelocity(),
                Constants.Shooter.State.tolerance)
        ) return ShooterState.SHOOT_BOTTOM;

        return ShooterState.NONE;
    }

    @Override
    public void initDefaultCommand() {
        // Nothing
    }

    @Override
    public void displayTestData() {
        SmartDashboard.putNumber("Left spark max velocity", this._sparkMaxLeft.getVelocity());
        SmartDashboard.putNumber("Right spark max velocity", this._sparkMaxRight.getVelocity());
        // System.out.println("L: " + this._sparkMaxLeft.getVelocity()
        //     + ", R: " + this._sparkMaxRight.getVelocity());
        // SmartDashboard.putNumber("shooter diff", this._sparkMaxLeft.getVelocity() - this._sparkMaxRight.getVelocity());
    }

    @Override
    public void displayMatchData() {
        SmartDashboard.putBoolean("Shooter SHOOT", this.getShooterState() != ShooterState.NONE && Utils.isInNeighborhood(this._lastSpeed - (this._sparkMaxLeft.getVelocity() + this._sparkMaxRight.getVelocity()) / 2, 0.0, 10.0));
        this._lastSpeed = (this._sparkMaxLeft.getVelocity() + this._sparkMaxRight.getVelocity()) / 2;
    }

    @Override
    public void displayCommands() {
        SmartDashboard.putData("Shooter SHOOT", new NewSetShooterState(ShooterState.SHOOT));
        SmartDashboard.putData("Shooter NONE", new NewSetShooterState(ShooterState.NONE));
    }
}