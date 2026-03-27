package frc.robot.Subsystems.Drive.Vision;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Util.*;
import frc.robot.Util.LimelightHelpers.RawFiducial;
import frc.robot.RobotConstants;

public class LimelightIO implements CameraIO{

    private String camName;

    private Transform3d offset;
    // In Degrees
    private double yaw;
    private double kCamTiltDegrees = RobotConstants.VisionConstants().kCamTiltDegrees;

    public LimelightIO(String name, Transform3d cameraOffset) {
        // Instantiate a Limelight and account for position of limelight on the robot
        camName = name;
        // Set pipeline and other default settings for image processing
        LimelightHelpers.setPipelineIndex(camName, 0);
        this.offset = cameraOffset;
        LimelightHelpers.SetIMUAssistAlpha(camName, 0.001);
        LimelightHelpers.setLEDMode_PipelineControl(camName);
        LimelightHelpers.setCameraPose_RobotSpace(camName, offset.getX(), offset.getY(), offset.getZ(), 0, kCamTiltDegrees, 0);
    }

    public Optional<Pose2d> estimateBotPose(Pose2d position) {
        Optional<Pose2d> pose = Optional.ofNullable(position);
        return pose;
    }

    

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.camName = camName;
        inputs.isConnected = true; 

        // Checks if camera is still connected
        double newHeartbeat = LimelightHelpers.getHeartbeat(camName);
        if (!(newHeartbeat > inputs.oldHeartbeat)) {
            inputs.isConnected = false;
            inputs.hasBeenUpdated = false;
        } else {
            inputs.oldHeartbeat = newHeartbeat;
            inputs.isConnected = true;
            inputs.hasBeenUpdated = true;
        }
    
        // Get robot field pose with MegaTag2
        // Might need to take in a SwerveDrivePoseEstimator

        // Get pose estimate
        LimelightHelpers.SetRobotOrientation(camName, yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate visionResult = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName);

        // Get an Optional Pose2d to handle potential null values
        Optional<Pose2d> estPose = estimateBotPose(visionResult.pose);
        inputs.latestEstimatedRobotPose = estPose.orElse(new Pose2d(new Translation2d(Double.MAX_VALUE, Double.MAX_VALUE), new Rotation2d(Double.MAX_VALUE)));
        inputs.yaw = inputs.latestEstimatedRobotPose.getRotation().getRadians();
        inputs.latestTimestamp = visionResult.timestampSeconds;

        if (inputs.latestEstimatedRobotPose.getX() != Double.MAX_VALUE) {
            RawFiducial[] fiducialData = visionResult.rawFiducials;

            // Check how many tags and their distances to camera & ambiguities
            inputs.ambiguities =  new double[fiducialData.length];
            inputs.tags = new int[fiducialData.length];
            inputs.distances = new double[fiducialData.length];
            for (int i = 0; i < fiducialData.length; i++) {
                inputs.ambiguities[i] = fiducialData[i].ambiguity;   
                inputs.tags[i] = fiducialData[i].id;
                inputs.distances[i] = fiducialData[i].distToCamera;
            }
        } else {
            inputs.ambiguities =  new double[0];
            inputs.tags = new int[0];
            inputs.distances = new double[0];
        }
    }
}