package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    static class gyroInputs {
        public boolean isOK = false;
        public double rotationsYaw = 0.0d;
        public double rotationsRoll = 0.0d;
        public double rotationsPitch = 0.0d;
        public double rpsYaw = 0.0d;
        public double rpsRoll = 0.0d;
        public double rpsPitch = 0.0d;
    }

    default public void updateInputs(gyroInputs toUpdate) {}

    /**
     * Reads the angle of the gyroscope, the dummy layer always returns -1.0.
     * @return The current angle in rotations on the Z axis of the robot.
     */
    default public double getYawAngleRotations() { return -1.0d; }

    /**
     * Reads the change in angle of the gyroscope, the dummy layer always returns -1.0.
     * @return The change in angle in rotations per second on the Z axis of the robot.
     */
    default public double getYawAngleRPS() { return -1.0d; }

    /**
     * When extra processing is needed before updating the gyroscope, this method is really
     * only used to update the sim gyroscope however, since most real gyroscopes actually work.
     * @param omega The change in angle between this update and the previous update (This should be calculated via swerve odometry).
     * @param dt The change in time.
     */
    default public void updateYaw(double omega, double dt) {}

    /**
     * Resets the gyroscope reading to zero (Currently unsused because I don't want to set up commands just yet).
     */
    default public void resetGyro() {}
}
