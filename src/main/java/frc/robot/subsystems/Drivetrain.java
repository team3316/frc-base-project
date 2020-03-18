package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team3316.kit.DBugSubsystem;
import com.team3316.kit.control.PIDFGains;
import com.team3316.kit.motors.DBugTalon;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.autonomous.StartingConfig;
import frc.robot.utils.Utils;

/**
 * Drivetrain
 */
public class Drivetrain extends DBugSubsystem {

    // L - left, R - Right, F - front, B - back
    private DBugTalon _talonL, _talonR;
    private VictorSPX _victorLF, _victorRF, _victorLB, _victorRB;
    private PigeonIMU _pigeon;
    private DoubleSolenoid _shifter;
    
    // Auto
    public DifferentialDriveKinematics DriveKinematics;
    private DifferentialDriveOdometry _odometry;

    private void configureTalon(DBugTalon talon, boolean inverted, boolean sensorPhase, PIDFGains gainsDefault, PIDFGains gainsTurn) {
        talon.setInverted(inverted);
        talon.setSensorPhase(sensorPhase);

        talon.setupPIDF(gainsDefault.getP(), gainsDefault.getI(), gainsDefault.getD(), gainsDefault.getF(), 0);
        talon.configAllowableClosedloopError(0, (int) gainsDefault.getTolerance(), 0);

        talon.setupPIDF(gainsTurn.getP(), gainsTurn.getI(), gainsTurn.getD(), gainsTurn.getF(), 1);
        talon.configAllowableClosedloopError(0, (int) gainsTurn.getTolerance(), 1);
    }

    public Drivetrain() {
        
        DriveKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.wheelDistance);

        /*
         * Motor controller definitions
         */
        this._talonR = (DBugTalon) Utils.getBean("drivetrain.talonR");
        this._talonL = (DBugTalon) Utils.getBean("drivetrain.talonL");
        this._victorRB = (VictorSPX) Utils.getBean("drivetrain.victorRB");
        this._victorLB = (VictorSPX) Utils.getBean("drivetrain.victorLB");
        this._victorRF = (VictorSPX) Utils.getBean("drivetrain.victorRF");
        this._victorLF = (VictorSPX) Utils.getBean("drivetrain.victorLF");

        this._talonL.configNeutralDeadband(0.05);
        this._talonR.configNeutralDeadband(0.05);

        /*
         * Resetting motor controllers to defaults
         */
        
        this._victorRB.configFactoryDefault();
        this._victorLB.configFactoryDefault();
        this._victorRF.configFactoryDefault();
        this._victorLF.configFactoryDefault();
        /*
         * Defining followers
         */

        this._victorLB.follow(this._talonL);
        this._victorLF.follow(this._talonL);

        this._victorRB.follow(this._talonR);
        this._victorRF.follow(this._talonR);
        /*
         * Talon configuration
         */

        configureTalon(this._talonL, Constants.Drivetrain.MotorControllers.TalonL.inverted,
                Constants.Drivetrain.MotorControllers.TalonL.sensorPhase,
                Constants.Drivetrain.MotorControllers.TalonL.gainsDefault,
                Constants.Drivetrain.MotorControllers.TalonL.gainsTurn);

        configureTalon(this._talonR, Constants.Drivetrain.MotorControllers.TalonR.inverted,
                Constants.Drivetrain.MotorControllers.TalonR.sensorPhase,
                Constants.Drivetrain.MotorControllers.TalonR.gainsDefault,
                Constants.Drivetrain.MotorControllers.TalonR.gainsTurn);


        this._victorRF.setInverted(Constants.Drivetrain.MotorControllers.VictorRF.inverted);
        
        this._victorRB.setInverted(Constants.Drivetrain.MotorControllers.VictorRB.inverted);
        
        this._victorLF.setInverted(Constants.Drivetrain.MotorControllers.VictorLF.inverted);

        this._victorLB.setInverted(Constants.Drivetrain.MotorControllers.VictorLB.inverted);

        /*
         * Encoder talons configuration
         */
        //TODO: Actually measure these
        this._talonL.setDistancePerRevolution(Constants.Drivetrain.DPR, Constants.Sensors.Encoders.Quad.upr);
        this._talonR.setDistancePerRevolution(Constants.Drivetrain.DPR, Constants.Sensors.Encoders.Quad.upr);
        
        /*
         * Pigeon definition
         */
        this._pigeon = (PigeonIMU) Utils.getBean("drivetrain.pigeon");

        /*
         * Shifter definition
         */
        this._shifter = (DoubleSolenoid) Utils.getBean("drivetrain.shifter");

        this._odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYawDegrees()));

        /*
         * Resetting
         */
        this.resetStartingConfig(Constants.Autonomous.startingConfig);

    }

    public double getVelocityInRawUnits() {
        return this._talonL.getSelectedSensorVelocity();
    }
    public double getLeftMotorOutput() {
        return this._talonL.getMotorOutputPercent();
    }

    
    public double getRightMotorOutput() {
        return this._talonR.getMotorOutputPercent();
    }

    /**
     * Sets sensors to a given starting config
     * @param startingConfig a startingConfig object containing the starting configuration of all relevant sensors
     */
    public void resetStartingConfig(StartingConfig startingConfig) {
        this.setEncDistance(startingConfig.getLeftDistance(), startingConfig.getRightDistance());
        this.setYaw(startingConfig.getYaw());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return this._odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(this._talonL.getVelocity(), this._talonR.getVelocity());
    }

    public void setDistance(double distL, double distR) {
        this._talonL.set(ControlMode.Position, distL);
        this._talonR.set(ControlMode.Position, distR);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        // DBugLogger.getInstance().info("Setting voltage. right: " + -rightVolts + ", left: " + leftVolts);
        this._talonL.set(ControlMode.PercentOutput, leftVolts / RobotController.getBatteryVoltage());
        this._talonR.set(ControlMode.PercentOutput, rightVolts / RobotController.getBatteryVoltage());
    }

    public void updateOdometry() {
        this._odometry.update(Rotation2d.fromDegrees(getYawDegrees()),
            getLeftDistance(),
            getRightDistance());
    }

    /**
     * This function returns the left encoder's distance
     * 
     * @return the left encoder's distance.
     */
    public double getLeftDistance() {
        return this._talonL.getDistance();
        
        
    }

    /**
     * This function returns the right encoder's distance
     * 
     * @return the right encoder's distance.
     */
    public double getRightDistance() {
        return this._talonR.getDistance();
    }

    /**
     * This function returns the robot center's distance
     * 
     * @return the robot center's distance.
     */
    public double getCenterDistance() {
        return (this.getRightDistance() + this.getLeftDistance()) / 2;
    }

    /**
     * This function sets the left encoder's distance
     * 
     * @param distance - the distance we want to set.
     */
    public void setLeftEncDistance(double distance) {
        this._talonL.setDistance(distance);
    }

    /**
     * This function sets the right encoder's distance
     * 
     * @param distance - the distance we want to set.
     */
    public void setRightEncDistance(double distance) {
        this._talonR.setDistance(distance);
    }

    /**
     * This function sets the both encoders' distance
     * 
     * @param distanceL - the distance we want to set the left encoder to.
     * @param distanceR - the distance we want to set the right encoder to.
     */
    public void setEncDistance(double distanceL, double distanceR) {
        this.setLeftEncDistance(distanceL);
        this.setRightEncDistance(distanceR);
    }

    /**
     * This function sets the both encoders' distance
     * 
     * @param distance - the distance we want to set both encoders to.
     */
    public void setEncDistance(double distance) {
        this.setLeftEncDistance(distance);
        this.setRightEncDistance(distance);
    }

    /**
     * Returns the heading of the robot.
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getYawDegrees() {
        double[] ypr = new double[3];
        this._pigeon.getYawPitchRoll(ypr);
        return Math.IEEEremainder(ypr[0], 360) * (Constants.Drivetrain.gyroReversed ? -1.0 : 1.0);
    }

    /**
     * This function sets the left side motors' percent output
     * 
     * @param percent - the percentOutput we want to set - between -1 and 1.
     */
    public void setLeftPercent(double percent) {
        this._talonL.set(ControlMode.PercentOutput, percent);
    }

    /**
     * This function sets the right side motors' percent output
     * 
     * @param percent - the percentOutput we want to set - between -1 and 1.
     */
    public void setRightPercent(double percent) {
        this._talonR.set(ControlMode.PercentOutput, percent);
    }

    /**
     * This function sets both sides motors' percent output
     * 
     * @param percentL - the percentOutput we want to set the left side to - between -1
     *                 and 1.
     * @param percentR - the percentOutput we want to set the right side to - between -1
     *                 and 1.
     */
    public void setPercent(double percentL, double percentR) {
        this.setLeftPercent(percentL);
        this.setRightPercent(percentR);
    }

    /**
     * This function sets both sides motors percent output
     * 
     * @param percent - the percentOutput  we want to set both sides to.
     */
    public void setPercent(double percent) {
        this.setLeftPercent(percent);
        this.setRightPercent(percent);
    }

    /**
     * This function sets the left side motors' position
     * 
     * @param position - the position we want to set im meters.
     */
    public void setLeftPosition(double position) {
        this._talonL.set(ControlMode.Position, position);
    }
    
    /**
     * This function sets the right side motors' position
     * 
     * @param position - the position we want to set in meters.
     */
    public void setRightPosition(double position) {
        this._talonR.set(ControlMode.Position, position);
    }

    /**
     * This function sets both sides motors' position
     * 
     * @param positionL - the posotion we want to set the left side to in meters.
     * @param positionR - the position we want to set the right side to in meters.
     *                  and 1.
     */
    public void setPosition(double positionL, double positionR) {
        this.setLeftPosition(positionL);
        this.setRightPosition(positionR);
    }

    /**
     * This function sets both sides motors position
     * 
     * @param position - the position we want to set both sides to in meters.
     */
    public void setPosition(double position) {
        this.setLeftPosition(position);
        this.setRightPosition(position);
    }

    /**
     * This function sets the left side motors' velocity
     * 
     * @param velocity - the velocity we want to set in m/s
     */
    public void setLeftVelocity(double velocity) {
        this._talonL.set(ControlMode.Velocity, velocity);
    }

    /**
     * This function sets the right side motors' velocity
     * 
     * @param velocity - the velocity we want to set in m/s.
     */
    public void setRightVelocity(double velocity) {
        this._talonR.set(ControlMode.Velocity, velocity);
    }

    /**
     * This function sets both sides motors' velocity
     * 
     * @param velocityL - the velocity we want to set the left side to in m/s.
     * @param velocityR - the velocity we want to set the right side to in m/s.
     *                  and 1.
     */
    public void setVelocity(double velocityL, double velocityR) {
        this.setLeftVelocity(velocityL);
        this.setRightVelocity(velocityR);
    }

    /**
     * This function sets both sides motors position
     * 
     * @param velocity - the velocity we want to set both sides to in m/s.
     */
    public void setVelocity(double velocity) {
        this.setLeftVelocity(velocity);
        this.setRightVelocity(velocity);
    }


    /**
     * Sets the robot's yaw angle in degrees.
     * 
     * @param yaw the robot's yaw angle we want to set to.
     */
    public void setYaw(double yaw) {
        this._pigeon.setYaw(yaw);
    }

    /**
     * Sets the motors' neutral state.
     * 
     * @param status whether to make the neautral state of the left motor brake or not.
     */
    public void setBrake(boolean status) {
        NeutralMode mode = status ? NeutralMode.Brake : NeutralMode.Coast;
        BaseMotorController[] motorControllers = new BaseMotorController[] {
            this._talonL, this._victorLB, this._victorLF,
            this._talonR, this._victorRB, this._victorRF
        };
        for (int i = 0; i < motorControllers.length; i++) {
            motorControllers[i].setNeutralMode(mode);
        }

    }

    public enum Gear {
        HIGH(Value.kForward), LOW(Value.kReverse), OFF(Value.kOff);

        private Value _value;

        private Gear(Value value) {
            this._value = value;
        }

        public Value getValue() {
            return this._value;
        }
    }

    /**
     * get's the drivetrain's gear.
     * 
     * @return the current gear to set to.
     */
    public Gear getCurrentGear() {
        Value state = this._shifter.get();

        switch (state) {
        case kReverse:
            return Gear.LOW;

        case kForward:
            return Gear.HIGH;

        default:
            return Gear.OFF;
        }
    }

    /**
     * Set's the drivetrain's gear.
     * 
     * @param gear - the gear to set to.
     */
    public void setGear(Gear gear) {
        this._shifter.set(gear.getValue());
    }

    @Override
    public void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

    @Override
    public void displayTestData() {
        SmartDashboard.putBoolean("HIGH GEAR", this.getCurrentGear() == Gear.HIGH);        
    }

    @Override
    public void displayMatchData() {
        SmartDashboard.putBoolean("Drivetrain SLOW", this.getCurrentGear() == Gear.LOW);   

    }

    @Override
    public void displayCommands() {
        
    }
}