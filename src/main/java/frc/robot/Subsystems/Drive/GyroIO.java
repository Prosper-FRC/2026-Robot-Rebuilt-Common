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
}
