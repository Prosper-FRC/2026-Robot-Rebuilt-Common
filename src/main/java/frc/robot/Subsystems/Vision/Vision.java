package frc.robot.Subsystems.Vision;

import static frc.robot.Subsystems.Vision.visionConstants.VisionConstants.kAmbiguityThreshold;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;



public class Vision extends SubsystemBase {
    private CameraIO camera;
    private CameraIOInputsAutoLogged cameraData;

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
    

    public record VisionObservation(boolean hasObserved, Pose2d pose, double timeStamp, boolean isValid) {}

    // public void logVisionObservation(VisionObservation observation) {
    //     Logger.recordOutput("Vision/Observation/hasObserved", observation.hasObserved());
    //     Logger.recordOutput("Vision/Observation/pose", observation.pose());
    //     Logger.recordOutput("Vision/Observation/timeStamp", observation.timeStamp());
    //     Logger.recordOutput("Vision/Observation/isValid", observation.isValid());
    // }
}
