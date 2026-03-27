package frc.robot.Subsystems.Drive.Vision;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.Vision.CameraIO.LEDMode;
import frc.robot.Util.LimelightHelpers;



public class Vision extends SubsystemBase {
    private CameraIO camera;
    private CameraIOInputsAutoLogged cameraData;
    private String camName = RobotConstants.VisionConstants().kCamName;

    private final double kAmbiguityThreshold = RobotConstants.VisionConstants().kAmbiguityThreshold;

    @AutoLogOutput(key = "Vision/pose")
    private Pose2d Observation = new Pose2d();

    @AutoLogOutput(key = "Vision/Ok")
    private boolean ok = false; 

    @AutoLogOutput(key = "Vision/isValid")
    private boolean isValid = false; 

    private final AprilTagFieldLayout k2026Field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    public Vision(CameraIO camera) {
        this.camera = camera;
        cameraData = new CameraIOInputsAutoLogged();
    }
    
    public void periodic() {     
            camera.updateInputs(cameraData); 
            Logger.processInputs("Vision", cameraData);  
            Observation = cameraData.latestEstimatedRobotPose;
            ok = cameraData.isConnected;
            VisionObservation observation = getVisionObservation();
            isValid = observation.isValid();
    }

    // Check reliability of vision
    public VisionObservation getVisionObservation() {

        VisionObservation observation = new VisionObservation(false, null, 0, false);
        // Check tag length
        if (cameraData.tags.length == 0) {
            observation = new VisionObservation(
                true, 
                cameraData.latestEstimatedRobotPose,
                cameraData.latestTimestamp, 
                false
            );
        }
        else if (cameraData.tags.length == 1) {
            if (cameraData.ambiguities[0] > kAmbiguityThreshold) {
                observation = new VisionObservation(
                    true, 
                    cameraData.latestEstimatedRobotPose, 
                    cameraData.latestTimestamp,
                    false
                );
            } else {
                observation = new VisionObservation(
                    true, 
                    cameraData.latestEstimatedRobotPose, 
                    cameraData.latestTimestamp,
                    true
                );
            }
        } 
        
        else if (cameraData.tags.length > 1) {
            for (int i = 0; i < cameraData.tags.length; i++) {
                if (cameraData.ambiguities[i] > kAmbiguityThreshold) {
                    observation = new VisionObservation(
                        true, 
                        cameraData.latestEstimatedRobotPose, 
                        cameraData.latestTimestamp,
                        false
                    );
                    
                    break;
                } 
            }
        } 
        else if (
            cameraData.latestEstimatedRobotPose.getX() < 0.0 || 
            cameraData.latestEstimatedRobotPose.getX() > k2026Field.getFieldLength() ||
            cameraData.latestEstimatedRobotPose.getY() < 0.0 ||
            cameraData.latestEstimatedRobotPose.getY() > k2026Field.getFieldWidth()
            ) {
                observation = new VisionObservation(
                    true, 
                    cameraData.latestEstimatedRobotPose, 
                    cameraData.latestTimestamp,
                    false
                );

        } else if (cameraData.yaw == Double.MAX_VALUE) {
            observation = new VisionObservation(
                true, 
                cameraData.latestEstimatedRobotPose, 
                cameraData.latestTimestamp,
                false
            );
        }
        // If below ambiguity threshold and within dimension in field
        else {
            observation = new VisionObservation(
            true, 
            cameraData.latestEstimatedRobotPose, 
            cameraData.latestTimestamp,
            true
            );
        }
        return observation;
    }
    
    public Pose2d getPose() {
        return cameraData.latestEstimatedRobotPose;
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

    // Yaw in degrees
    public void setYaw(double yaw) {
        cameraData.yaw = yaw;
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

    public record VisionObservation(boolean hasObserved, Pose2d pose, double timeStamp, boolean isValid) {}

    // public void logVisionObservation(VisionObservation observation) {
    //     Logger.recordOutput("Vision/Observation/hasObserved", observation.hasObserved());
    //     Logger.recordOutput("Vision/Observation/pose", observation.pose());
    //     Logger.recordOutput("Vision/Observation/timeStamp", observation.timeStamp());
    //     Logger.recordOutput("Vision/Observation/isValid", observation.isValid());
    // }
}
