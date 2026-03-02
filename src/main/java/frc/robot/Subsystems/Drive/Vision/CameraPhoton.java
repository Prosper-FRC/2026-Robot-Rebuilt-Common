package frc.robot.Subsystems.Drive.Vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotConstants;

public class CameraPhoton {
    @AutoLog
    static class cameraInputs {
        public boolean isConnected = false;

        public double fieldPoseXMeters = 0.0d;
        public double fieldPoseYMeters = 0.0d;
        public double fieldPoseZMeters = 0.0d;
        public double fieldAngleRadians = 0.0d;
        public double distanceFromTarget = 0.0d;
        public int tagId = -1;
        public double tagAmbiguity = 0.0d;
    }

    static class VisionReadings {
        public ArrayList<Pose2d> poses;
        public ArrayList<Double> timestamps;

        public VisionReadings(ArrayList<Pose2d> poses, ArrayList<Double> timestamps) {
            this.poses = poses;
            this.timestamps = timestamps;
        }
        public VisionReadings() {
            this.poses = new ArrayList<>();
            this.timestamps = new ArrayList<>();
        }

        public void addReading(Pose2d pose, Double timestamp) {
            poses.add(pose);
            timestamps.add(timestamp);
        }

        public Pair<Pose2d, Double> getReading(int index) {
            return new Pair<Pose2d,Double>(poses.get(index), timestamps.get(index));
        }

        public void combineReadings(VisionReadings combineReadings) {
            for(var otherPose : combineReadings.poses) {
                poses.add(otherPose);
            }
            for(var otherTimestamp : combineReadings.timestamps) {
                timestamps.add(otherTimestamp);
            }
        }
    }

    private final PhotonCamera kCamera;
    public final cameraInputsAutoLogged kInputs = new cameraInputsAutoLogged();
    private final Pose3d kTransform;
    public final String kName;

    private List<PhotonPipelineResult> results;

    public CameraPhoton(String cameraName, Pose3d cameraTransform) {
        kCamera = new PhotonCamera(cameraName);
        kTransform = cameraTransform;
        kName = cameraName;
        results = kCamera.getAllUnreadResults();
        updateInputs(kInputs);
    }

    public void updateInputs(cameraInputs inputs) {
        if(results != null) {
            var latestBestTarget = results.get(0).getBestTarget();

            inputs.isConnected = kCamera.isConnected();
            inputs.tagId = latestBestTarget.fiducialId;
            inputs.tagAmbiguity = latestBestTarget.getPoseAmbiguity();
            var relativePosition = latestBestTarget.getBestCameraToTarget();
            Pose3d fieldPosition = PhotonUtils.estimateFieldToRobotAprilTag(
                relativePosition, 
                RobotConstants.CameraConstants().kLayout.getTagPose(latestBestTarget.fiducialId).get(), 
                new Transform3d(kTransform.getX(), kTransform.getY(), kTransform.getZ(), kTransform.getRotation()));
            inputs.fieldPoseXMeters = fieldPosition.getX();
            inputs.fieldPoseYMeters = fieldPosition.getY();
            inputs.fieldPoseZMeters = fieldPosition.getZ();

            inputs.distanceFromTarget = PhotonUtils.calculateDistanceToTargetMeters(
                RobotConstants.CameraConstants().kCameraToRobotTranslation.getZ(), 
                RobotConstants.CameraConstants().kLayout.getTagPose(latestBestTarget.fiducialId).get().getZ(),
                RobotConstants.CameraConstants().kCameraAngleRotation.getY(), 
                0.0d);

            inputs.fieldAngleRadians = fieldPosition.getRotation().getY();
        }
    }

    public void updateVisionReadings() {
        results = kCamera.getAllUnreadResults();
    }

    // TODO Flip based on alliance, currently results are always based on blue.
    public VisionReadings getVisionReadings() {
        VisionReadings transforms = new VisionReadings(); // Stores both the transform and the timestamp.
        for(var result : results) {
            // Uses MT Results.
            result.getMultiTagResult().ifPresent((MTresult) -> {
                var resultTranslation = MTresult.estimatedPose.best;
                Pose2d resultAsPose = new Pose2d(resultTranslation.getX(), resultTranslation.getY(), resultTranslation.getRotation().toRotation2d());
                transforms.addReading(resultAsPose, result.getTimestampSeconds());
            });
        }
        return transforms;
    }
}