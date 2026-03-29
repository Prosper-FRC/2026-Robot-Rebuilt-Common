package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class flywheelInputs {
        public boolean isOk = false;
        public double velocityRPS = 0.0d;
        public double appliedVoltage = 0.0d;
        public double statorCurrent = 0.0d;
        public double supplyCurrent = 0.0d;
    }

    default public void updateInputs(flywheelInputs toUpdate) {}

    default public void setFlywheelRPS(double rps) {}

    default public void setFlywheelVoltage(double voltage) {}

    default public void stopFlywheel() {}
}
