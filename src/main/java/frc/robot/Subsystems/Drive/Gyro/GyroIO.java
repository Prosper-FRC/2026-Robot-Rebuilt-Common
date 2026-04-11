package frc.robot.Subsystems.Drive.Gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public class GyroInputs {
        public boolean gyroOk = false;
        public double gyroPositionRotations = 0.0d;
        public double gyroSpeedRotationPerSeconod = 0.0d;
    }

    default public void updateInputs(GyroInputs inputs) {}

    default public void resetGyro() {}
    default public void resetGyro(double rotation) {}

    default public void updateGyro(double rotations) {}
}
