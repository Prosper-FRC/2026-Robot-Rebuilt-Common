package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
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

        public boolean CANCoderOk = false;
    }

    default public void updateInputs(moduleInputs toUpdate) {}

    // Drive methods

    /**
     * Sets the target drive position in rotations (Not implemented into sim).
     * @param rotations The number of rotations.
     */
    default public void setDriveRotations(double rotations) {}

    /**
     * Sets the target drive velocity in rotations per second.
     * @param rps The number of rotations per second to target.
     */
    default public void setDriveRPS(double rps) {}

    /**
     * Sets the voltage applied to the drive motor.
     * @param volts The voltage to apply to the drive motor.
     */
    default public void setDriveVoltage(double volts) {}

    /**
     * Stops the motor.
     */
    default public void stopDrive() {}

    /**
     * Sets the encoder reading of the drive motor to zero.
     */
    default public void resetDrive() {}

    // Azimuth methods

    /**
     * Sets the target position of the azimuth in rotations.
     * @param rotations The number of rotations to target.
     */
    default public void setAzimuthRotations(double rotations) {}

    /**
     * Sets the target velocity of the azimuth in rotations per second (Not implemented into sim).
     * @param rps The number of rotations per second to target.
     */
    default public void setAzimuthRPS(double rps) {}

    /**
     * Sets the voltage applied to the azimuth motor.
     * @param volts The voltage applied to the azimuth motor.
     */
    default public void setAzimuthVoltage(double volts) {}

    /**
     * Sets the azimuth's position to the CANCoders absolute raw reading
     */
    default public void recalibrateAzimuth() {}

    /**
     * Stops the motor.
     */
    default public void stopAzimuth() {}

    // Misc

    /**
     * Updates the PID values for the drive motor configuration.
     * @param kP The P term.
     * @param kI The I term.
     * @param kD The D term.
     */
    default public void updateDrivePIDValues(double kP, double kI, double kD) {}

    /**
     * Updates the PID values for the azimuth motor configuration.
     * @param kP The P term.
     * @param kI The I term.
     * @param kD The D term.
     */
    default public void updateAzimuthPIDValues(double kP, double kI, double kD) {}
}
