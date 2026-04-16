package frc.robot.Subsystems.Drive.Gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public class GyroInputs {
        public boolean gyroOk = false;
        public double gyroPositionRotations = 0.0d;
        public double gyroSpeedRotationPerSeconod = 0.0d;
    }

    /**
     * Updates NT logging data from given inputs.
     * @param inputs The inputs to update
     */
    default public void updateInputs(GyroInputs inputs) {}

    /**
     * Resets the gyro reading to zero.
     */
    default public void resetGyro() {}
    /**
     * Resets the gyro reading to a given rotation.
     * @param rotation The angle in rotations to set the gyro
     */
    default public void resetGyro(double rotation) {}

    /**
     * Changes the gyro by a given number of rotations (SIM ONLY).
     * @param rotations The angle in rotations to change the gyro by.
     */
    default public void updateGyro(double rotations) {}
}
