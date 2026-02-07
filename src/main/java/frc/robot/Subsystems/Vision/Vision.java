package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.Vision.CameraIO.CameraIOInputs;
import frc.robot.utils.debugging.LoggedTunableNumber; // Unsure how to add the utils folder, if it is any different from 2025 reefscape
import static frc.robot.Subsystems.Vision.VisionConstants.kSingleStdDevs;
import static frc.robot.Subsystems.Vision.VisionConstants.kMultiStdDevs;
import static frc.robot.Subsystems.Vision.VisionConstants.KUseSingleTagTransform;
import static frc.robot.Subsystems.Vision.VisionConstants.kAmbiguityThreshold;;


public class Vision {
    private CameraIO[] cameras;
    private CameraIOInputsAutoLogged[] camerasData;
    
    // FOR LIMELIGHTS ONLY
    private boolean isLimelight = false;
    private final CameraIOInputsAutoLogged inputsRight = new CameraIOInputsAutoLogged();
    private final CameraIOInputsAutoLogged inputsLeft = new CameraIOInputsAutoLogged();

    private static final LoggedTunableNumber kSingleXYStdev = new LoggedTunableNumber(
        "Vision/kSingleXYStdev", kSingleStdDevs.get(0));
    private static final LoggedTunableNumber kMultiXYStdev = new LoggedTunableNumber(
        "Vision/kMultiXYStdev", kMultiStdDevs.get(0));

    private final AprilTagFieldLayout k2026Field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // PhotonVision
    public Vision(CameraIO[] cameras) {
        Logger.recordOutput("Vision/UseSingleTagTransform", KUseSingleTagTransform);
        this.cameras = cameras;
        camerasData = new CameraIOInputsAutoLogged[cameras.length];
        for(int i = 0; i < cameras.length; i++) {
            camerasData[i] = new CameraIOInputsAutoLogged();
        }
    }

    public void periodic(Pose2d lastRobotPose, Pose2d simOdomPose) {
        
        for(int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(camerasData[i], lastRobotPose, simOdomPose);
            Logger.processInputs("Vision/"+camerasData[i].camName, camerasData[i]);
            // Logger.recordOutput("Vision/"+camerasData[i].camName+"/Pose", camerasData[i].latestEstimatedRobotPose.toPose2d());
            // Logger.recordOutput("Vision/"+camerasData[i].camName+"/X", camerasData[i].latestEstimatedRobotPose.getRotation().getX());
            // Logger.recordOutput("Vision/"+camerasData[i].camName+"/Y", camerasData[i].latestEstimatedRobotPose.getRotation().getY());
            // Logger.recordOutput("Vision/"+camerasData[i].camName+"/Z", camerasData[i].latestEstimatedRobotPose.getRotation().getZ());
        }

    }


    // Gets the vision data. Standard Deviations are how much we trus the vision value
    public VisionObservation[] getVisionObservations() {
        VisionObservation[] observations = new VisionObservation[cameras.length];
        int i = 0;
        // STANDARD DEVIATION CALCULATIONS \\
        for(CameraIOInputsAutoLogged camData : camerasData) {
            // No point in adding vision data if it doesn't exist
            if(camData.hasTarget && camData.hasBeenUpdated) {
                // Average distance from tag, and the number of tags to determine estimate stability
                double numberOfTargets = camData.numberOfTargets;
                double avgDistMeters = 0.0;
                for(int r = 0; r < camData.latestTagTransforms.length; r++) {
                    if(camData.latestTagTransforms[r] != null) {
                        if(camData.latestTagAmbiguities[r] < kAmbiguityThreshold) {
                            avgDistMeters += camData.latestTagTransforms[r].getTranslation().getNorm();
                        } else {
                            numberOfTargets -= 1;
                        }
                    }
                }

                // No point in adding vision data if it doesn't exist(as all the tags were to ambiguos to trust)
                if(numberOfTargets == 0) {
                    observations[i] = new VisionObservation(
                        true, 
                        camData.latestEstimatedRobotPose.toPose2d(), 
                        /* Max std devs indicate the data can't be trusted */
                        VecBuilder.fill(
                            Double.MAX_VALUE, 
                            Double.MAX_VALUE, 
                            Double.MAX_VALUE), 
                        camData.latestTimestamp, camData.camName);

                    i++;
                    continue;
                }

                avgDistMeters /= numberOfTargets;
                // Logger.recordOutput("Vision/AvgDistMeters", avgDistMeters);

                double xyScalar = Math.pow(avgDistMeters, 2) / (numberOfTargets);

                // Logger.recordOutput("Vision/xyScalar", xyScalar);

                // Cases where we shouldn't add vision measurements
                if(numberOfTargets == 1 && avgDistMeters > 3.5) {
                    observations[i] = new VisionObservation(
                        true,
                        camData.latestEstimatedRobotPose.toPose2d(), 
                        /* Max std devs indicate the data can't be trusted */
                        VecBuilder.fill(
                            Double.MAX_VALUE, 
                            Double.MAX_VALUE, 
                            Double.MAX_VALUE), 
                        camData.latestTimestamp, camData.camName);
                // In other cases, run single-tag calibration
                } else if(numberOfTargets == 1) {
                    Pose2d singleTagPose = new Pose2d();
                    if(KUseSingleTagTransform) {
                        singleTagPose = 
                            // Pose of involved tag
                            k2026Field.getTagPose(camData.singleTagAprilTagID).get().toPose2d()
                            // Transform pose to camera
                            .plus(new Transform2d(
                                    camData.cameraToApriltag.getX(), camData.cameraToApriltag.getY(), 
                                    camData.cameraToApriltag.getRotation().toRotation2d()))
                            // Transform camera to robot center
                            .plus(toTransform2d(camData.cameraToRobot.inverse()));
                    } else {
                        singleTagPose = camData.latestEstimatedRobotPose.toPose2d();
                    }
                    observations[i] = new VisionObservation(
                        true,
                        singleTagPose, 
                        VecBuilder.fill(
                            kSingleXYStdev.get() * xyScalar, 
                            kSingleXYStdev.get() * xyScalar, 
                            Double.MAX_VALUE), 
                        camData.latestTimestamp, camData.camName);
                // In other cases, run multi-tag calibration
                } else {
                    observations[i] = new VisionObservation(
                        true,
                        camData.latestEstimatedRobotPose.toPose2d(), 
                        VecBuilder.fill(
                            kMultiXYStdev.get() * xyScalar, 
                            kMultiXYStdev.get() * xyScalar, 
                            Double.MAX_VALUE), 
                        camData.latestTimestamp, camData.camName);

                }
            } else {
                observations[i] = new VisionObservation(
                    false, 
                    new Pose2d(), 
                    /* Max std devs indicate the data can't be trusted */
                    VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE), 
                    camData.latestTimestamp, camData.camName);
            }
            i++;
       }
        return observations;
    }

    private Transform2d toTransform2d(Transform3d transform) {
        return new Transform2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
    }

    public void logVisionObservation(VisionObservation observation, String state) {
        Logger.recordOutput("Vision/Observation/"+observation.camName+"/State", state);
        Logger.recordOutput("Vision/Observation/"+observation.camName+"/Timestamp", observation.camName());
        Logger.recordOutput("Vision/Observation/"+observation.camName+"/Pose", observation.pose());
        Logger.recordOutput("Vision/Observation/"+observation.camName+"/hasObserved", observation.hasObserved());
        Logger.recordOutput("Vision/Observation/"+observation.camName+"/StdDevs", observation.stdDevs());
    }

    public record VisionObservation(boolean hasObserved, Pose2d pose, Vector<N3> stdDevs, double timeStamp, String camName) {}
}