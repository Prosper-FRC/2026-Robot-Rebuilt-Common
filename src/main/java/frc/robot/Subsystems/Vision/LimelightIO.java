package frc.robot.Subsystems.Vision;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Vision.VisionConstants.Orientation;;

public class LimelightIO implements CameraIO{

    private String camName;
    private Transform3d cameraTransform;
    private Orientation orientation;

    private NetworkTable limelight;
    private boolean pipelineIndex;
    private double[] poseValues;
    private Pose3d offset;
    private double verticalDistance;
    private double sidewaysDistance;
    private double forwardDistance;
    private double roll;
    private double pitch;
    private double yaw;

    ArrayList<Transform3d> tagTargets = new ArrayList<>();

    public LimelightIO(String name, Pose3d offset, Transform3d cameraTransform, Orientation orientation) {
        // Instantiate a Limelight and account for position of limelight on the robot
        camName = name;
        this.cameraTransform = cameraTransform;
        limelight = NetworkTableInstance.getDefault().getTable(camName);
        // Set pipeline for image processing
        limelight.getEntry("pipeline").setNumber(0);
        this.offset = offset;
        this.orientation = orientation;
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

    // Gets the target apriltag in front of limelight
    public int getTargetID() {
        return (int) limelight.getEntry("tid").getInteger(100);
    }

    // Check if limelight has a target
    public boolean hasTarget() {
        return (limelight.getEntry("tv").getDouble(0) == 1);
    }


    public Pose3d getPose() {
        // pose values of robot position relative to the blue driverstation
        poseValues = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[10]);

        Translation3d translate = new Translation3d(poseValues[0], poseValues[1], poseValues[2]);
        Rotation3d rotate = new Rotation3d(Math.toRadians(poseValues[3]), Math.toRadians(poseValues[4]), Math.toRadians(poseValues[5]));

        return new Pose3d(translate, rotate);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs, Pose2d lastRobotPose, Pose2d simOdomPose) {
        // Get target by getting entry of the target (apriltag) and accounting for the offset of the limelights
        poseValues = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        // [2] Gets tz (forward horizontal distance), and [0] gets tx (sideways horizontal distance); tx is made 
        // negative to fit into WPILib's coordinate system from Limelight Camera Space coordinate system (BOTH coordinate systems)
        // Consider one same direction as two different signs
        verticalDistance = -poseValues[1];
        forwardDistance = poseValues[2];
        sidewaysDistance = -poseValues[0];
        pitch = Math.toRadians(poseValues[3]);
        yaw = Math.toRadians(poseValues[4]);
        roll = Math.toRadians(poseValues[5]);
        Rotation3d rotation = new Rotation3d(roll, pitch, yaw);
        Transform3d transform = new Transform3d(forwardDistance, sidewaysDistance, verticalDistance, rotation); 
        
        // Add cl and tl for total latency
        inputs.latencySeconds = (limelight.getEntry("tl").getDouble(0) + limelight.getEntry("cl").getDouble(0));
        inputs.cameraToApriltag = transform;
        // Accounts for robot position
        inputs.robotToApriltag = transform.plus(cameraTransform);
        inputs.singleTagAprilTagID = getTargetID();
        inputs.pitch = pitch;
        inputs.yaw = yaw;
        inputs.pitch = pitch;
        inputs.area = getArea();

        if (orientation.equals(Orientation.FRONT)) {
            inputs.latestEstimatedRobotPose = getPose();
        } else {
            // Rotate pose by 180 degrees or PI radians
            inputs.latestEstimatedRobotPose = getPose().transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI)));
        }

        inputs.latestTimestamp = Timer.getFPGATimestamp() - inputs.latencySeconds;

        // Photon Vision code from Reefscape Common; planning to take the logic of the code below and adapt it for limelight

        // ArrayList<Transform3d> tagTs = new ArrayList<>();
        //                 double[] ambiguities = new double[latestEstimatedRobotPose.get().targetsUsed.size()];
        //                 if(latestEstimatedRobotPose.get().targetsUsed.size() > 0) {
        //                     for(int i = 0; i < latestEstimatedRobotPose.get().targetsUsed.size(); i++) {
        //                         tagTs.add(latestEstimatedRobotPose.get().targetsUsed.get(i).getBestCameraToTarget());
        //                         ambiguities[i] = latestEstimatedRobotPose.get().targetsUsed.get(i).getPoseAmbiguity();
        //                     }
        //                 }
   
    }


}
