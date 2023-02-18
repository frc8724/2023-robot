package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {

    public LimeLight() {
    }

    @Override
    public void periodic() {
        update();
    }

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // x location of the target
    private NetworkTableEntry tx = table.getEntry("tx");
    // y location of the target
    private NetworkTableEntry ty = table.getEntry("ty");
    // area of the target
    private NetworkTableEntry ta = table.getEntry("ta");
    // does the limelight have a target
    private NetworkTableEntry tv = table.getEntry("tv");

    public double getX() {
        return tx.getDouble(0.0);
    }

    public double getY() {
        return ty.getDouble(0.0);
    }

    public double getDistance() {
        double heightOfCamera = 43;
        double heightOfTarget = 29;
        double angleOfCamera = -20;
        double angleofTarget = ty.getDouble(0.0);
        return (heightOfTarget - heightOfCamera) / Math.tan(Math.toRadians(angleOfCamera + angleofTarget));
    }

    public double getArea() {
        return ta.getDouble(0.0);
    }

    public boolean isTargetAvalible() {
        return tv.getDouble(0.0) > 0.5;
    }

    // Moding the limelight to work for the individual case
    public void drivingMode(boolean driveMode) {
        ledMode(!driveMode);
        double mode = driveMode ? 1 : 0;
        table.getEntry("camMode").setNumber(mode);
    }

    public void ledMode(boolean on) {
        double mode = on ? 0 : 1;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
    }

    public void pipelineMode(int pipeLmode) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeLmode);
    }

    public enum VisionModes {
        LOW(0),
        LEFT(1),
        RIGHT(2);

        public double mode;

        VisionModes(double mode) {
            this.mode = mode;
        }
    }

    public void setVisionMode() {
        setVisionMode(VisionModes.LOW);
    }

    public void setVisionMode(VisionModes visionMode) {
        table.getEntry("pipeline").setNumber(visionMode.mode);
    }

    public static String getVisionMode() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getString("0");
    }

    double[] getTargetPose() {
        return NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("botpose_targetspace")
                .getDoubleArray(new double[6]);
    }

    double getTargetPoseId() {
        return NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("tid")
                .getDouble(-1.0);
    }

    /**
     * Get distance from April Tag in meters
     * 
     * @return
     */
    public double getDistanceFromAprilTag() {
        double[] d = getTargetPose();
        return Math.abs(d[2]);
    }

    int count = 0;

    public void update() {
        SmartDashboard.putNumber("limelight x", getX());
        SmartDashboard.putNumber("limelight y", getY());
        SmartDashboard.putNumber("limelight area", getArea());
        SmartDashboard.putNumber("limelight distance", getDistance());
        SmartDashboard.putBoolean("limelight has target2", isTargetAvalible());
        SmartDashboard.putString("limelight mode", getVisionMode());
        SmartDashboard.putNumber("limelight count", count++);

        double[] d = getTargetPose();
        SmartDashboard.putNumber("LimeLight Pose 0", d[0]);
        SmartDashboard.putNumber("LimeLight Pose 1", d[1]);
        SmartDashboard.putNumber("LimeLight Pose 2", d[2]); // distance from target
        SmartDashboard.putNumber("LimeLight Pose 3", d[3]);
        SmartDashboard.putNumber("LimeLight Pose 4", d[4]);
        SmartDashboard.putNumber("LimeLight Pose 5", d[5]);

        SmartDashboard.putNumber("LimeLight Pose Id", getTargetPoseId());
        SmartDashboard.putNumber("LimeList Distance to April", getDistanceFromAprilTag());
    }

}