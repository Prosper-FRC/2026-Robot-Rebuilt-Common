package frc.robot.Subsystems.Drive.Vision;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;

public interface CameraIO {
    @AutoLog
    static class VisionInputs {
        public boolean isOk = false;
        public double fieldPoseEstimationX = 0.0d;
        public double fieldPoseEstimationY = 0.0d;
        public double fieldPoseEstimationYaw = 0.0d;
    }

    /**
     * LED Mode override
     */
    public static enum LEDMode {
        DEFAULT,
        ON,
        OFF,
        BLINK
    }

    /**
     * A special method that returns all null values and acts as a proper dummy layer.
     */
    public static final CameraIO NoOp = new CameraIO() {};

    /**
     * Gets the field pose estimation through PhotonLibs prebuilt methods taking CameraToPose offset into account.
     * @return An optional representing the field pose estimation.
     */
    default public Optional<Pose2d> getFieldPoseEstimation() {
        return Optional.empty();
    }

    /**
     * Configures the camera to pose offset, this method is robot relative. This method is taken into account when getting the field pose estimation.
     * @param poseOffset The translation from the center of the camera to the center of the robot.
     */
    default public void setCameraToPoseOffset(Translation3d poseOffset) {}

    /**
     * Allows you to supply the ids you want to be taken into account in vision. If no values are set than all tags will be taken into account.
     * @param ids The ids you want to take into account.
     */
    default public void setTagFilters(int... ids) {}

    /**
     * Allows you to set the screen scale of the processed image, only use this method if you need a higher framerate and are willing to sacrafice result quality.
     * @param screenScale The proportion of the screen to use in the range <strong>0.0 < screenScale <= 1.0</strong>.
     */
    default public void screenScale(float screenScale) {}

    /**
     * Sets the current LED mode allowing you to control the mode of the LED. Only use this if you intend to implement manual LED control.
     * @param mode
     */
    default public void setLEDMode(LEDMode mode) {}

    /**
     * Captures a snapshot of what the limelight sees.
     */
    default public void captureSnapshot() {}

    /**
     * Rewind is a Limelight 4 feature that allows the recording of what it sees during matches.
     * @param useRewind A boolean turing limelight on and off.
     */
    default public void setRewindMode(boolean useRewind) {}

    /**
     * Captures footage that was recorded.
     * @param seconds how many seconds of previous footage to save.
     */
    default public void captureSeconds(double seconds) {}

    /**
     * Sets the IMU mode to the pre calibration mode (Mode 1: EXTERNAL_SEED). This mode calibrates the internal IMU of the Limelight 4 for fusion with the onboard IMU down the line.
     */
    default public void setIMUModePreCalibration() {}

    /**
     * Uitlizes the onboard gyro to correct for translational drift at 50hz while the 1khz Limelight 4's internal IMU quickly updates to provide the most accurate measurements.
     */
    default public void setIMUModeAssist() {}

    /**
     * Uitlizes the onboard gyro to correct for translational drift at 50hz while the 1khz Limelight 4's internal IMU quickly updates to provide the most accurate measurements.
     * @param IMUAssistAlpha Adjusts the correction speed of the onboard gyro, the default value is 0.001.
     */
    default public void setIMUModeAssist(double IMUAssistAlpha) {}

    /**
     * Utilizes only the onboard gyro along with vision readings for pose estimation. This method needs to be used if the Limelight version is not 4.0, this caps the update rate of the gyro to 50hz which can hurt precision as the vision readings inbetween the 50hz update rate will not have an accurate angle supplied at higher angular velocities.
     */
    default public void setIMUModeExternalOnly() {}

    /**
     * Gives the Limelight access to the gyro reading for a more accurate application of MT2. It is recommended you supply the gyro reading through the pose estimator since it will also blend the vision results with the reading allowing the gyro's rotational offset to be corrected over time.
     * @param yaw A double supplier to the gyro's yaw reading in <strong>degrees</strong>.
     */
    default public void supplyRobotAngleYaw(DoubleSupplier yaw) {}

    /**
     * Loads a new Limelight pipeline to be used.
     * @param pipelineIndex The index of the pipeline in use.
     */
    default public void loadPipeline(int pipelineIndex) {}
}
