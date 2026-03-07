package frc.robot.Subsystems.vision;

import static frc.robot.Subsystems.vision.visionConstants.VisionConstants.kAmbiguityThreshold;


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
import frc.robot.utils.debugging.LoggedTunableNumber; // Unsure how to add the utils folder, if it is any different from 2025 reefscape;


import frc.robot.Subsystems.vision.CameraIOInputsAutoLogged;

import static frc.robot.Subsystems.vision.visionConstants.VisionConstants.kAmbiguityThreshold;
import frc.robot.Subsystems.vision.CameraIO.CameraIOInputs;

public class Vision {
    private CameraIO camera;
    private CameraIOInputsAutoLogged cameraData;

    private final AprilTagFieldLayout k2026Field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public Vision(CameraIO camera) {
        // Logger.recordOutput("Vision/UseSingleTagTransform", KUseSingleTagTransform);
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
        VisionObservation observation = new VisionObservation(false, false, null, 0);
        if (cameraData.tags.length == 0) {
            observation = new VisionObservation(
                true, 
                true,
                cameraData.latestEstimatedRobotPose,
                cameraData.latestTimestamp
            );
        }
        else if (cameraData.tags.length == 1) {
            if (cameraData.ambiguities[0] > kAmbiguityThreshold) {
                observation = new VisionObservation(
                    true, 
                    true,
                    cameraData.latestEstimatedRobotPose, 
                    cameraData.latestTimestamp
                );
            }
        } else if (cameraData.tags.length > 1) {
            for (int i = 0; i < cameraData.tags.length; i++) {
                if (cameraData.ambiguities[i] > kAmbiguityThreshold) {
                    observation = new VisionObservation(
                        true, 
                        false,
                        cameraData.latestEstimatedRobotPose, 
                        cameraData.latestTimestamp
                    );
                }
            }
        }
        return observation;
    }

    public record VisionObservation(boolean hasObserved, boolean isRejected, Pose2d pose, double timeStamp) {}
}

