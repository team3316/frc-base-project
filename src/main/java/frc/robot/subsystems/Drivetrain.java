package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team3316.kit.DBugSubsystem;
import com.team3316.kit.control.PIDFGains;
import com.team3316.kit.motors.DBugTalon;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;
import frc.robot.utils.Utils;

/**
 * Drivetrain
 */
public class Drivetrain extends DBugSubsystem {
    private DBugTalon _talonL, _talonR;
    private VictorSPX _victorLF, _victorRF, _victorLB, _victorRB;
    private BaseMotorController[] motorControllers = {
        this._talonL, this._victorLB, this._victorLF,
        this._talonR, this._victorRB, this._victorRF
    };

    private PigeonIMU _pigeon;
    
    // For path following
    public DifferentialDriveKinematics DriveKinematics;
    private DifferentialDriveOdometry _odometry;

    public Drivetrain() {    
        /*
         * Motor controller initialization
         */
        this._talonR = (DBugTalon) Utils.getBean("drivetrain.talonR");
        this._talonL = (DBugTalon) Utils.getBean("drivetrain.talonL");
        this._victorRB = (VictorSPX) Utils.getBean("drivetrain.victorRB");
        this._victorLB = (VictorSPX) Utils.getBean("drivetrain.victorLB");
        this._victorRF = (VictorSPX) Utils.getBean("drivetrain.victorRF");
        this._victorLF = (VictorSPX) Utils.getBean("drivetrain.victorLF");

        configureMaster(this._talonR,
                        Constants.Drivetrain.MotorControllers.Right.invertType,
                        Constants.Drivetrain.MotorControllers.Right.sensorPhase,
                        Constants.Drivetrain.DPR,
                        Constants.Drivetrain.MotorControllers.Right.gainsMain);

        configureSlave(this._victorRB, this._talonR);
        configureSlave(this._victorLB, this._talonR);

        configureMaster(this._talonL,
                        Constants.Drivetrain.MotorControllers.Left.invertType,
                        Constants.Drivetrain.MotorControllers.Left.sensorPhase,
                        Constants.Drivetrain.DPR,
                        Constants.Drivetrain.MotorControllers.Left.gainsMain);

        configureSlave(this._victorRF, this._talonL);
        configureSlave(this._victorLF, this._talonL);

        /*
         * Pigeon definition
         */
        this._pigeon = (PigeonIMU) Utils.getBean("drivetrain.pigeon");

        // For path following
        DriveKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.wheelDistance);
        this._odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYawDegrees()));
    }

    //// CONFIGURING MOTOR CONTROLLERS

    private void configureMaster(DBugTalon talon,
                                 InvertType invertType,
                                 boolean sensorPhase,
                                 double DPR,
                                 PIDFGains gains_0) {
        talon.setInverted(invertType);
        talon.setSensorPhase(sensorPhase);
        talon.setDistancePerRevolution(DPR, Constants.Sensors.Encoders.Quad.upr);
        // TODO check if PID works when gains are set only for master and not for slaves
        // TODO add a PIDFGains constructor with int instead of double for tolerance in DBugKit
        talon.setupPIDF(gains_0.getP(), gains_0.getI(), gains_0.getD(), gains_0.getF(), 0);
        talon.configAllowableClosedloopError(0, (int) gains_0.getTolerance(), 0);
    }

    private void configureSlave(BaseMotorController slave, DBugTalon master) {
        slave.follow(master);
        slave.setInverted(InvertType.FollowMaster);
    }

    //// GET ////

    /**
     * @return The Right Motor's distance [Measured in Meters assuming DPR is set accordingly]
     */
    public double getRightDistance() {
        return this._talonR.getDistance();
    }

    /**
     * @return The Left Motor's distance [Measured in Meters assuming DPR is set accordingly]
     */
    public double getLeftDistance() {
        return this._talonL.getDistance();
    }

    /**
     * @return The Right Motor Controller's output in percentage [-1 <= output <= 1]
     */
    public double getRightMotorOutput() {
        return this._talonR.getMotorOutputPercent();
    }

    /**
     * @return The Left Motor Controller's output in percentage [-1 <= output <= 1]
     */
    public double getLeftMotorOutput() {
        return this._talonL.getMotorOutputPercent();
    }

    /**
     * @return The Right Motor's velocity [Measured in Meters per 100ms assuming DPR is set accordingly]
     */
    public double getRightVelocity() {
        return this._talonR.getVelocity();
    }

    /**
     * @return The Left Motor's velocity [Measured in Meters per 100ms assuming DPR is set accordingly]
     */
    public double getLeftVelocity() {
        return this._talonL.getVelocity();
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

    //// SET ////

    /**
     * Sets all Motor Controllers' brake state
     * @param setToBrake brake when true, coast when false
     */
    public void setBrake(boolean setToBrake) {
        NeutralMode mode = setToBrake ? NeutralMode.Brake : NeutralMode.Coast;
        for (int i = 0; i < motorControllers.length; i++) motorControllers[i].setNeutralMode(mode);
    }

    /**
     * Sets both motors' velocity
     * @param percentage Desired velocity [M/Sec]
     */
    public void setVelocity(double velocity) {
        this._talonR.set(ControlMode.Velocity, velocity);
        this._talonL.set(ControlMode.Velocity, velocity);
    }

    /**
     * Sets both motors' velocity
     * @param velocityR Desired Right velocity [M/Sec]
     * @param velocityL Desired Left velocity [M/Sec]
     */
    public void setVelocity(double velocityR, double velocityL) {
        this._talonR.set(ControlMode.Velocity, velocityR);
        this._talonL.set(ControlMode.Velocity, velocityL);
    }

    /**
     * Sets both motors' percentage output
     * @param percentage [-1 <= percentage <= 1]
     */
    public void setPercentage(double percentage) {
        this._talonR.set(ControlMode.PercentOutput, percentage);
        this._talonL.set(ControlMode.PercentOutput, percentage);
    }

    /**
     * Sets both motors' percentage output
     * @param percentageR [-1 <= percentageR <= 1]
     * @param percentageL [-1 <= percentageL <= 1]
     */
    public void setPercentage(double percentageR, double percentageL) {
        this._talonR.set(ControlMode.PercentOutput, percentageR);
        this._talonL.set(ControlMode.PercentOutput, percentageL);
    }

    /**
     * Sets the Right encoder's value
     * @param dist The value to set to [Same units as set in DPR]
     */
    public void setRightEncoderDistance(double dist) {
        this._talonR.setDistance(dist);
    }

    /**
     * Sets the Left encoder's value
     * @param dist The value to set to [Same units as set in DPR]
     */
    public void setLeftEncoderDistance(double dist) {
        this._talonL.setDistance(dist);
    }

    /**
     * Sets both encoders' value
     * @param dist The value to set to [Same units as set in DPR]
     */
    public void setEncoderDistance(double dist) {
        setRightEncoderDistance(dist);
        setLeftEncoderDistance(dist);
    }

    /**
     * Sets both encoders' value
     * @param distR The value to set the Right encoder to [Same units as set in DPR]
     * @param distl The value to set the Left encoder to [Same units as set in DPR]
     */
    public void setEncoderDistance(double distR, double distL) {
        setRightEncoderDistance(distR);
        setLeftEncoderDistance(distL);
    }

    /**
     * Sets the robot's yaw angle in degrees.
     * @param yaw the robot's yaw angle we want to set to.
     */
    public void setYaw(double yaw) {
        this._pigeon.setYaw(yaw);
    }

    //// PATH FOLLOWING ////

    /**
     * Returns the currently-estimated pose of the robot.
     * @return The pose
     */
    public Pose2d getPose() {
        return this._odometry.getPoseMeters();
    }

    /**
     * Needed for path following.
     * @return The current wheel speeds as a DifferentialDriveWheelSpeeds object
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }

    /**
     * Needed for path following, controls the left and right sides of the drive directly with voltages.
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        // this._talonL.set(ControlMode.PercentOutput, leftVolts / RobotController.getBatteryVoltage());
        // this._talonR.set(ControlMode.PercentOutput, rightVolts / RobotController.getBatteryVoltage());
    }

    /**
     * Needed for path following, Update Odometry object with new data from sensors.
     */
    public void updateOdometry() {
        // this._odometry.update(Rotation2d.fromDegrees(getYawDegrees()),
        //     getLeftDistance(),
        //     getRightDistance());
    }

    //// DEFAULT COMMAND ////

    @Override
    public void initDefaultCommand() {
        // Nothing
    }

    //// SDB PRINTS ////

    @Override
    public void displayTestData() {
        // Nothing
    }

    @Override
    public void displayMatchData() {
        // Nothing
    }

    @Override
    public void displayCommands() {
        // Nothing
    }
}