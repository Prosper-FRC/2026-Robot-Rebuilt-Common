package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;;

public class LimelightIO implements CameraIO{

    private String camName;
    // Not used in anything for now
    private Transform3d cameraTransform;

    private NetworkTable limelight;
    private boolean pipelineIndex;
    private double[] poseValues;
    private Pose3d offset;

    public LimelightIO(String name, Pose3d offset, Transform3d cameraTransform) {
        // Instantiate a Limelight and account for position of limelight on the robot
        camName = name;
        this.cameraTransform = cameraTransform;
        limelight = NetworkTableInstance.getDefault().getTable(camName);
        // Set pipeline for image processing
        limelight.getEntry("pipeline").setNumber(0);
        this.offset = offset;
    }

    public double getYaw() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getPitch() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public double getArea() {
        return limelight.getEntry("ta").getDouble(0);
    }

    public void setPipelineIndex(int index) {
        limelight.getEntry("pipeline").setNumber(index);
    }

    public int getPipelineIndex() {
        return pipelineIndex ? 1 : 0;
    }

    // Gets the target apriltag in front of limelight
    public long getTargetID() {
        return limelight.getEntry("tid").getInteger(100);
    }

    // Check if limelight has a target
    public boolean hasTarget() {
        return (limelight.getEntry("tv").getDouble(0) == 1);
    }

    // Get target by getting entry of the target (apriltag) and accounting for the offset of the limelights
    public Pose2d getTarget() {
        poseValues = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        Translation2d translate = new Translation2d(poseValues[0] - offset.getX(), poseValues[1] - offset.getY());
        Rotation2d rotation = new Rotation2d(Math.toRadians(poseValues[3]));
        return new Pose2d(translate, rotation);
    }

    public Pose2d getPose() {
        // pose values of robot position relative to the blue driverstation
        poseValues = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        Translation2d translate = new Translation2d(poseValues[0], poseValues[1]);
        Rotation2d rotate = Rotation2d.fromDegrees(poseValues[3]);

        return new Pose2d(translate, rotate);
    }

    public void updateInputs(CameraIOInputs inputs) {
        // Update camera to target pose
        inputs.cameraToTarget = getTarget();
        inputs.yaw = limelight.getEntry("tx").getDouble(0.0);
        inputs.pitch = limelight.getEntry("ty").getDouble(0.0);
        inputs.area = limelight.getEntry("ta").getDouble(0.0);
        // Add cl and tl for total latency
        inputs.latencySeconds = (limelight.getEntry("tl").getDouble(0) + limelight.getEntry("cl").getDouble(0));
    }


}
