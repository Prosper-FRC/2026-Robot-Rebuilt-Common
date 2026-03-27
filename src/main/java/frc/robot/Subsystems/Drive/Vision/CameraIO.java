package frc.robot.Subsystems.Drive.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {

        public String camName = "";
        public boolean isConnected = false;
        public double yaw = 0.0;
        public double oldHeartbeat = 0.0;

        public double latestTimestamp = 0.0;
        public boolean hasBeenUpdated = false;
        public Pose2d latestEstimatedRobotPose;
        public double[] ambiguities = new double[0];
        public int[] tags = new int[0];
        public double[] distances = new double[0];
    }
        public default void updateInputs(CameraIOInputs inputs) {}

        public static enum LEDMode {
            DEFAULT, ON, OFF, BLINK
        }
}