package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface FlywheelIO {
    @AutoLog
    static class FlywheelInputs {
        public boolean flywheelOk = false;
        public double flywheelPositionRotations = 0.0d;
        public double flywheelVelocityRPM = 0.0d;
        public double flywheelTemperatureCelcius = 0.0d;
        public double flywheelVoltage = 0.0d;
        public double flywheelStatorCurrent = 0.0d;
        public double flywheelSupplyCurrent = 0.0d;
    }

    default public void updateInputs(FlywheelInputs toUpdate) {} 

    default public void setFlywheelVoltage(double volts) {}

    default public void setFlywheelVelocity(double velocity) {}

    default public void stopFlywheel() {}

    default public void resetFlywheel() {}
}