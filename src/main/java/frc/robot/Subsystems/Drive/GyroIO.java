package frc.robot.Subsystems.Drive;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    static class gyroInputs {
        public boolean isOk = false;
        public double rollRotations = 0.0d;
        public double pitchRotations = 0.0d;
        public double yawRotations = 0.0d;
    }

    /**
     * Updates the inputs for AK logging.
     * @param toUpdate the inputs to update
     */
    default public void updateInputs(gyroInputs toUpdate) {}

    /**
     * Changes the yaw of the gyro reading by a delta value.
     * @param yaw The number of rotations to change the yaw by.
     */
    default public void updateGyro(double yaw) {}

    /**
     * Sets the yaw of the gyro reading to a value.
     * @param yaw The number of rotations to set the yaw to.
     */
    default public void setGyro(double yaw) {}

    /**
     * Resets the gyro reading on the z-axis (the yaw)
     */
    default public void resetGyro() {}

    default public Optional<Rotation2d> getGyroAngle() {
        return Optional.empty();
    }
}
