package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public String camName = "";
        public boolean isConnected = false;
        public double yaw = 0.0;
        public double pitch = 0.0;
        public double area = 0.0;
        public double latencySeconds = 0.0;
        public boolean hasTarget = false;
        public int numberOfTargets = 0;

        public Transform3d cameraToRobot = new Transform3d();
        public Transform3d cameraToApriltag = new Transform3d();
        public double poseAmbiguity = 0.0;
        public int singleTagAprilTagID = 0;
        public Transform3d robotToApriltag = new Transform3d();
        public double latestTimestamp = 0.0;
        public boolean hasBeenUpdated = false;
        public Pose3d latestEstimatedRobotPose = new Pose3d();
        /* Array size equal to amount of tags */
        public Transform3d[] latestTagTransforms = new Transform3d[] {};
        public double[] latestTagAmbiguities = new double[] {};
    }

    public default void updateInputs(CameraIOInputs inputs, Pose2d lastRobotPose, Pose2d simOdomPose) {}
}