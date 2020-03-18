package frc.robot.autonomous;

/**
 * StartingConfig
 */
public class StartingConfig {
    public static StartingConfig exampleStartingConfig = new StartingConfig(0, 0, 0);

    private double _yaw;
    private double _distanceL;
    private double _distanceR;

    private StartingConfig(double yaw, double distanceL, double distanceR) {

    }

    public double getYaw() {
        return this._yaw;
    }

    
    public double getLeftDistance() {
        return this._distanceL;
    }
    
    public double getRightDistance() {
        return this._distanceR;
    }
}