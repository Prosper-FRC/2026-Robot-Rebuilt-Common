package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    // TODO check unit conversion accuracy for ALL AK classes
    @AutoLog
    static class moduleInputs {
        public boolean driveOk = false;
        public double drivePositionRotations = 0.0d;
        public double driveVelocityRPS = 0.0d;
        public double driveTemperatureCelcius = 0.0d;
        public double driveStatorCurrent = 0.0d;
        public double driveSupplyCurrent = 0.0d;
        public double driveSupplyVoltage = 0.0d;

        public boolean azimuthOk = false;
        public double azimuthPositionRotations = 0.0d;
        public double azimuthVelocityRPS = 0.0d;
        public double azimuthTemperatureCelcius = 0.0d;
        public double azimuthStatorCurrent = 0.0d;
        public double azimuthSupplyCurrent = 0.0d;
        public double azimuthSupplyVoltage = 0.0d;
    }

    default public void updateInputs(moduleInputs toUpdate) {}

    // Drive methods
    default public void setDriveRotations(double rotations) {}

    default public void setDriveRPS(double rps) {}

    default public void setDriveVoltage(double volts) {}

    default public void stopDrive() {}

    default public void resetDrive() {}

    // Azimuth methods
    default public void setAzimuthRotations(double rotations) {}

    default public void setAzimuthRPS(double rps) {}

    default public void setAzimuthVoltage(double volts) {}

    default public void stopAzimuth() {}

    default public void resetAzimuth(double position) {}

    // Misc
    default public void updateDrivePIDValues(double kP, double kI, double kD) {}

    default public void updateAzimuthPIDValues(double kP, double kI, double kD) {}
}
