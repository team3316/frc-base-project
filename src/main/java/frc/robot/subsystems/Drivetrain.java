package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.team3316.kit.DBugSubsystem;

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
    private PigeonIMU _pigeon;
    
    // For path following
    public DifferentialDriveKinematics DriveKinematics;
    private DifferentialDriveOdometry _odometry;

    public Drivetrain() {    
        /*
         * Pigeon definition
         */
        this._pigeon = (PigeonIMU) Utils.getBean("drivetrain.pigeon");

        // For path following
        DriveKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.wheelDistance);
        this._odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYawDegrees()));
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

    //// GYRO ////
    
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
     * Sets the robot's yaw angle in degrees.
     * @param yaw the robot's yaw angle we want to set to.
     */
    public void setYaw(double yaw) {
        this._pigeon.setYaw(yaw);
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