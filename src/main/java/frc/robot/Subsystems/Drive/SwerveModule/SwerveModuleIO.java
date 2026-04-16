package frc.robot.Subsystems.Drive.SwerveModule;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleInputs {
        public boolean driveModuleOk = false;
        public double driveModuleSupplyVoltage = 0.0d;
        public double driveModuleMotorVoltage = 0.0d;
        public double driveModuleSupplyCurrent = 0.0d;
        public double driveModuleStatorCurrent = 0.0d;
        public double driveModulePositionRotations = 0.0d;
        public double driveModuleSpeedRotationsPerSecond = 0.0d;
        public double driveModuleAccelerationRotationsPerSecondSquared = 0.0d;
        
        public boolean azimuthModuleOk = false;
        public double azimuthModuleSupplyVoltage = 0.0d;
        public double azimuthModuleMotorVoltage = 0.0d;
        public double azimuthModuleSupplyCurrent = 0.0d;
        public double azimuthModuleStatorCurrent = 0.0d;
        public double azimuthModulePositionRotations = 0.0d;
        public double azimuthModuleSpeedRotationsPerSecond = 0.0d;
        public double azimuthModuleAccelerationRotationsPerSecondSquared = 0.0d;

        public boolean cancoderOk = false;
        public double cancoderPositionRotations = 0.0d;
        public double cancoderOffsetPositionRotations = 0.0d;
        public double cancoderSpeedRotationsPerSecond = 0.0d;
    }

    /**
     * Updates NT logging data from given inputs.
     * @param inputs The inputs to update
     */
    default public void updateInputs(SwerveModuleInputs inputs) {}

    // Drive methods //

    /**
     * Sets the output speed of the drive motor using voltage control mode.
     * @param rotationsPerSecond The speed to set the drive motor with units rotations per second.
     */
    default public void setDriveSpeedWithVoltage(double rotationsPerSecond) {}

    /**
     * Sets the output speed of the drive motor using FOC current control mode.
     * @param rotationsPerSecond The speed to set the drive motor with units rotations per second.
     */
    default public void setDriveSpeedWithFOC(double rotationsPerSecond) {}

    /**
     * Applies a voltage directly to the motor without any closed loop PID control.
     * @param motorVoltage The voltage to apply to the drive motor
     */
    default public void setDriveMotorVoltage(double motorVoltage) {}

    /**
     * Stops the drive motor and sets the control mode to neutral.
     */
    default public void stopDriveMotor() {}

    // Azimuth Methods //

    /**
     * Sets the target position of the azimuth motor using voltage control mode.
     * @param rotationsPerSecond The position to set the azimuth motor with units rotations.
     */
    default public void setAzimuthPositionWithVoltage(double rotations) {}

    /**
     * Sets the target position of the azimuth motor using FOC current control mode.
     * @param rotationsPerSecond The position to set the azimuth motor with units rotations.
     */
    default public void setAzimuthPositionWithFOC(double rotations) {}

    /**
     * Applies a voltage directly to the motor without any closed loop PID control.
     * @param motorVoltage The voltage to apply to the azimuth motor
     */
    default public void setAzimuthMotorVoltage(double motorVoltage) {}

    /**
     * Stops the azimuth motor and sets the control mode to neutral.
     */
    default public void stopAzimuthMotor() {}

    /**
     * Seeds the absolute encoder and applies offsets to it.
     */
    default public void seedAbsoluteEncoder() {}
}
