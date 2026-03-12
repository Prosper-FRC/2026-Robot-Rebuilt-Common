package frc.robot.Subsystems.vision;

import static frc.robot.Subsystems.vision.visionConstants.VisionConstants.kAmbiguityThreshold;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.vision.*;
import frc.robot.utils.debugging.LoggedTunableNumber; 


public class Vision {
    private CameraIO camera;
    private CameraIOInputsAutoLogged cameraData;

    private final AprilTagFieldLayout k2026Field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public Vision(CameraIO camera) {
        this.camera = camera;
        cameraData = new CameraIOInputsAutoLogged();
    }
    
     public void periodic() {
            camera.updateInputs(cameraData);
            Logger.processInputs("Vision/"+cameraData.camName, cameraData);
            Logger.recordOutput("Vision/"+cameraData.camName+"/Pose", cameraData.latestEstimatedRobotPose);
            Logger.recordOutput("Vision/"+cameraData.camName+"/Connected", cameraData.isConnected);
            Logger.recordOutput("Vision/"+cameraData.camName+"/VisibleTags", cameraData.tags);
            Logger.recordOutput("Vision/"+cameraData.camName+"/TagDistances", cameraData.distances);
    }

    // Check reliability of vision
    public VisionObservation getVisionObservations() {

        double hasAmbiguity = 0;

        VisionObservation observation = new VisionObservation(false, null, 0, false);
        if (cameraData.tags.length == 0) {
            observation = new VisionObservation(
                true, 
                cameraData.latestEstimatedRobotPose,
                cameraData.latestTimestamp, 
                false
            );
            hasAmbiguity++;
        }

        // Get scale factor to multiply with kXYStdDevs for actual standard deviation (not using STDDEVS, so comment out)

        // double avgDistMeters = 0;
        // for (int i = 0; i < cameraData.distances.length; i++) {
        //     avgDistMeters += cameraData.distances[i];
        // }
        // avgDistMeters /= cameraData.distances.length;
        // double xyScalar = Math.pow(avgDistMeters, 2) / (cameraData.tags.length);
        

        if (cameraData.tags.length == 1) {
            if (cameraData.ambiguities[0] > kAmbiguityThreshold) {
                observation = new VisionObservation(
                    true, 
                    cameraData.latestEstimatedRobotPose, 
                    cameraData.latestTimestamp,
                    false
                );
                hasAmbiguity++;
            }
        } else if (cameraData.tags.length > 1) {
            for (int i = 0; i < cameraData.tags.length; i++) {
                if (cameraData.ambiguities[i] > kAmbiguityThreshold) {
                    observation = new VisionObservation(
                        true, 
                        cameraData.latestEstimatedRobotPose, 
                        cameraData.latestTimestamp,
                        false
                    );
                    hasAmbiguity++;
                } 
            }
        }


        if (!(hasAmbiguity > 0)) {
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

    public void logVisionObservation(VisionObservation observation) {
        Logger.recordOutput("Vision/Observation/hasObserved", observation.hasObserved());
        Logger.recordOutput("Vision/Observation/pose", observation.pose());
        Logger.recordOutput("Vision/Observation/timeStamp", observation.timeStamp());
        Logger.recordOutput("Vision/Observation/isValid", observation.isValid());
    }
}
