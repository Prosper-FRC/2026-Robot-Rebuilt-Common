package frc.robot.Subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.LimelightHelpers.RawFiducial;

import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
       
        public Pose2d cameraToTarget = new Pose2d();

        public String camName = "";
        public boolean isConnected = false;
        public double yaw = 0.0;
        public double pitch = 0.0;
        public double latencySeconds = 0.0;
        public double oldHeartbeat = 0.0;
        public boolean hasTarget = false;

        public double latestTimestamp = 0.0;
        public boolean hasBeenUpdated = false;
        public Pose2d latestEstimatedRobotPose = new Pose2d();
        public double[] ambiguities = new double[0];
        public int[] tags = new int[0];
        public double[] distances = new double[0];
    }
        public default void updateInputs(CameraIOInputs inputs) {}

        public static enum LEDMode {
            DEFAULT, ON, OFF, BLINK
        }
}