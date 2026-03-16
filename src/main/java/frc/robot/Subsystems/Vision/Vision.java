package frc.robot.Subsystems.vision;

import static frc.robot.Subsystems.vision.visionConstants.VisionConstants.kAmbiguityThreshold;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;



public class Vision {
    private CameraIO camera;
    private CameraIOInputsAutoLogged cameraData;

    private final AprilTagFieldLayout k2026Field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    public Vision(CameraIO camera) {
        this.camera = camera;
        cameraData = new CameraIOInputsAutoLogged();
    }
    
     public void periodic() {
            Logger.processInputs("Vision/"+cameraData.camName, cameraData);
            camera.updateInputs(cameraData);   
            Logger.recordOutput("Vision/"+cameraData.camName+"/Observation", getVisionObservation());
            Logger.recordOutput("Vision/"+cameraData.camName+"/Pose", cameraData.latestEstimatedRobotPose);
            Logger.recordOutput("Vision/"+cameraData.camName+"/Connected", cameraData.isConnected);
            // Logger.recordOutput("Vision/"+cameraData.camName+"/VisibleTags", cameraData.tags);
            // Logger.recordOutput("Vision/"+cameraData.camName+"/TagDistances", cameraData.distances);
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

        // Get scale factor to multiply with kXYStdDevs for actual standard deviation (not using STDDEVS, so comment out)

        // double avgDistMeters = 0;
        // for (int i = 0; i < cameraData.distances.length; i++) {
        //     avgDistMeters += cameraData.distances[i];
        // }
        // avgDistMeters /= cameraData.distances.length;
        // double xyScalar = Math.pow(avgDistMeters, 2) / (cameraData.tags.length);
        
        // If valid tag amount, check ambiguity threshold
        else if (cameraData.tags.length == 1) {
            if (cameraData.ambiguities[0] > kAmbiguityThreshold) {
                observation = new VisionObservation(
                    true, 
                    cameraData.latestEstimatedRobotPose, 
                    cameraData.latestTimestamp,
                    false
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
