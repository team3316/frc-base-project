package frc.robot.vision;

import java.io.FileOutputStream;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionServer {

    private NetworkTable table;

    /**
     * Construcks a new 
     * @param cameraName The camera name which you want to get the input from.
     */
    public VisionServer(String cameraName) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("chameleon-vision").getSubTable(cameraName);
    }

    public void displayMatchData() {
        SmartDashboard.putBoolean("Go BACK", this.getNormalizeDistance() < 3.8);//actual distance 3.7m
        SmartDashboard.putBoolean("Go FRONT", this.getNormalizeDistance() > 4.2);//actaul distance 4.3m
    }

    // locate "yaw" in table and return it, if no infromation is found set to 3316
    public double getYawAngle() {
        return table.getEntry("targetYaw").getDouble(3316.0);
    }
    public double getTargetPitch() {
        return table.getEntry("targetPitch").getDouble(3316.0);
    }
    
    public double getTargetArea() {
        return table.getEntry("targetArea").getDouble(3316.0);
    }

    public double getTargetRectangleHeight() {
        return table.getEntry("targetBoundingHeight").getDouble(3316.0);
    }

    private double getYFOV() {
        return 33.6;
    }

    public double getNormalizeDistance() {
        final double opticalDistance = 1.3487 * this.getDistance() - 1.7219;
        return Math.sqrt((opticalDistance * opticalDistance) - (1.65 * 1.65)); 
    }

    private double getDistance(double rectHeight) {
        final double targetHeight = 0.42; // In Meters.
        final int yResolution = 240; // In Pixels.
        return (targetHeight * yResolution) / (2 * Math.tan(Math.toRadians(this.getYFOV()/2)) * rectHeight);
    }
    
    public double getDistance() {
        return this.getDistance(this.getTargetRectangleHeight());
    }

    public double[] getTargetPose() {
        double[] fakePose = {3316.0, 3316.0, 3316.0};
        return table.getEntry("targetPose").getDoubleArray(fakePose);
    }

    public double getActualPitch(){
        return (-1.2551*this.getTargetPitch()-0.05554);
    }

} 