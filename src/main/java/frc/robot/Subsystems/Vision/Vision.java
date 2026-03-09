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

    // PhotonVision
    public Vision(CameraIO cameras) {
        Logger.recordOutput("Vision/UseSingleTagTransform", KUseSingleTagTransform);
        this.cameras = cameras;
        camerasData = new CameraIOInputsAutoLogged();
        }
    }

    public void periodic(Pose2d lastRobotPose, Pose2d simOdomPose) {

            cameras.updateInputs(camerasData, lastRobotPose, simOdomPose);
            Logger.processInputs("Vision/"+camerasData.camName, camerasData);
            // Logger.recordOutput("Vision/"+camerasData[i].camName+"/Pose", camerasData[i].latestEstimatedRobotPose.toPose2d());
            // Logger.recordOutput("Vision/"+camerasData[i].camName+"/X", camerasData[i].latestEstimatedRobotPose.getRotation().getX());
            // Logger.recordOutput("Vision/"+camerasData[i].camName+"/Y", camerasData[i].latestEstimatedRobotPose.getRotation().getY());
            // Logger.recordOutput("Vision/"+camerasData[i].camName+"/Z", camerasData[i].latestEstimatedRobotPose.getRotation().getZ());
        }


    // Check reliability of vision
    public VisionObservation getVisionObservations() {
        // Just to make sure there is something returned
        VisionObservation observations = null;
        // STANDARD DEVIATION CALCULATIONS \\
        // No point in adding vision data if it doesn't exist
        if(camerasData.hasTarget && camerasData.hasBeenUpdated) {
            // Average distance from tag, and the number of tags to determine estimate stability
            double numberOfTargets = camerasData.numberOfTargets;
            double avgDistMeters = 0.0;
            for(int r = 0; r < camerasData.latestTagTransforms.length; r++) {
                if(camerasData.latestTagTransforms[r] != null) {
                    if(camerasData.latestTagAmbiguities[r] < kAmbiguityThreshold) {
                        avgDistMeters += camerasData.latestTagTransforms[r].getTranslation().getNorm();
                    } else {
                        numberOfTargets -= 1;
                    }
                }
            }

            // No point in adding vision data if it doesn't exist(as all the tags were to ambiguous to trust)
            if(numberOfTargets == 0) {
                observations = new VisionObservation(
                    true, 
                    camerasData.latestEstimatedRobotPose.toPose2d(), 
                    /* Max std devs indicate the data can't be trusted */
                    VecBuilder.fill(
                        Double.MAX_VALUE, 
                        Double.MAX_VALUE, 
                        Double.MAX_VALUE), 
                    camerasData.latestTimestamp, camerasData.camName);

                continue;
            }

            avgDistMeters /= numberOfTargets;
            // Logger.recordOutput("Vision/AvgDistMeters", avgDistMeters);

            double xyScalar = Math.pow(avgDistMeters, 2) / (numberOfTargets);

            // Logger.recordOutput("Vision/xyScalar", xyScalar);

            // Cases where we shouldn't add vision measurements
            if(numberOfTargets == 1 && avgDistMeters > 3.5) {
                observations = new VisionObservation(
                    true,
                    camerasData.latestEstimatedRobotPose.toPose2d(), 
                    /* Max std devs indicate the data can't be trusted */
                    VecBuilder.fill(
                        Double.MAX_VALUE, 
                        Double.MAX_VALUE, 
                        Double.MAX_VALUE), 
                    camerasData.latestTimestamp, camerasData.camName);
            // In other cases, run single-tag calibration
            } else if(numberOfTargets == 1) {
                Pose2d singleTagPose = new Pose2d();
                if(KUseSingleTagTransform) {
                    singleTagPose = 
                        // Pose of involved tag
                        k2026Field.getTagPose(camerasData.singleTagAprilTagID).get().toPose2d()
                        // Transform pose to camera
                        .plus(new Transform2d(
                                camerasData.cameraToApriltag.getX(), camerasData.cameraToApriltag.getY(), 
                                camerasData.cameraToApriltag.getRotation().toRotation2d()))
                        // Transform camera to robot center
                        .plus(toTransform2d(camerasData.cameraToRobot.inverse()));
                } else {
                    singleTagPose = camerasData.latestEstimatedRobotPose.toPose2d();
                }
                observations = new VisionObservation(
                    true,
                    singleTagPose, 
                    VecBuilder.fill(
                        kSingleXYStdev.get() * xyScalar, 
                        kSingleXYStdev.get() * xyScalar, 
                        Double.MAX_VALUE), 
                    camerasData.latestTimestamp, camerasData.camName);
            // In other cases, run multi-tag calibration
            } else {
                observations = new VisionObservation(
                    true,
                    camerasData.latestEstimatedRobotPose.toPose2d(), 
                    VecBuilder.fill(
                        kMultiXYStdev.get() * xyScalar, 
                        kMultiXYStdev.get() * xyScalar, 
                        Double.MAX_VALUE), 
                    camerasData.latestTimestamp, camerasData.camName);

            }
        } else {
            observations = new VisionObservation(
                false, 
                new Pose2d(), 
                /* Max std devs indicate the data can't be trusted */
                VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE), 
                camerasData.latestTimestamp, camerasData.camName);
        }
    return observations;
    }
    

    public record VisionObservation(boolean hasObserved, boolean isRejected, Pose2d pose, double timeStamp) {}


