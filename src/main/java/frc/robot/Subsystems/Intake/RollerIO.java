package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface RollerIO {
    @AutoLog
    static class RollerInputs {
        public boolean isOk = false;
        public double positionRotations = 0.0d;
        public double velocityRPS = 0.0d;
        public double temperatureCelcius = 0.0d;
        public double supplyVoltage = 0.0d;
        public double statorCurrent = 0.0d;
        public double supplyCurrent = 0.0d;
    }

    default public void updateInputs(RollerInputs toUpdate) {}

    default public void setOutputVoltage(double voltage) {}

    default public void setTargetVelocity(Rotation2d velocityPerSecond) {}

    default public void stopRollerMotor() {}
}
