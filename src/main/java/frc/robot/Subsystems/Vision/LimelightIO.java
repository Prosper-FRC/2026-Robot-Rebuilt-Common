package frc.robot.Subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class LimelightIO implements CameraIO{

    private String camName;

    private Transform3d offset;
    private double yaw;

    public LimelightIO(String name, Transform3d cameraOffset) {
        // Instantiate a Limelight and account for position of limelight on the robot
        camName = name;
        // Set pipeline and other default settings for image processing
        LimelightHelpers.setPipelineIndex(camName, 0);
        this.offset = cameraOffset;
        LimelightHelpers.SetIMUAssistAlpha(camName, 0.001);
        LimelightHelpers.setLEDMode_PipelineControl(camName);
        LimelightHelpers.setCameraPose_RobotSpace(camName, offset.getX(), offset.getY(), offset.getZ(), 0,0,0);
    }

    public void setTagFilters(int[] ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(camName, ids);
    }

    public void captureSnapshot() {
        LimelightHelpers.triggerSnapshot(camName);
    }

    public void setRewindMode() {
        LimelightHelpers.setRewindEnabled(camName, true);
    }

    public void captureSeconds(double seconds) {
        LimelightHelpers.triggerRewindCapture(camName, seconds);
    }

    public void loadPipeline(int index) {
        LimelightHelpers.setPipelineIndex(camName, index);
    }

    public void setIMUModePrecalibration() {
        LimelightHelpers.SetIMUMode(camName, 1);
    }
    
    public void setIMUModeExternalOnly() {
        LimelightHelpers.SetIMUMode(camName, 4);
    }

    public void setIMUModeAssist(double alphaLevel) {
        LimelightHelpers.SetIMUAssistAlpha(camName, alphaLevel);
    }

    public void setYaw(double yaw) {
        this.yaw = yaw;
    }

    public Optional<Pose2d> estimateBotPose(Pose2d position) {
        
        Optional<Pose2d> pose = Optional.ofNullable(position);
        return pose;
    }

    public void setLEDMode(LEDMode mode) {
        if(mode == LEDMode.ON) {
            LimelightHelpers.setLEDMode_ForceOn(camName);
        } else if(mode == LEDMode.OFF) {
            LimelightHelpers.setLEDMode_ForceOff(camName);
        } else if(mode == LEDMode.BLINK) {
            LimelightHelpers.setLEDMode_ForceBlink(camName);
        } else if (mode == LEDMode.DEFAULT) {}
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
        inputs.latestEstimatedRobotPose = estPose.orElse(new Pose2d(new Translation2d(9999,9999), new Rotation2d(Double.MAX_VALUE)));
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
            RawFiducial[] fiducialData = visionResult.rawFiducials;
            inputs.ambiguities =  new double[0];
            inputs.tags = new int[0];
            inputs.distances = new double[0];
        }
    }
}