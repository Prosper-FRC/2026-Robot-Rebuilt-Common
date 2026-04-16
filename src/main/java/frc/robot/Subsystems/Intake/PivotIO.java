package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {
    @AutoLog
    static class PivotInputs {
        public boolean isOk = false;
        public double positionRotations = 0.0d;
        public double velocityRPS = 0.0d;
        public double temperatureCelcius = 0.0d;
        public double supplyVoltage = 0.0d;
        public double statorCurrent = 0.0d;
        public double supplyCurrent = 0.0d;
    }

    /**
     * Updates NT logging data from given inputs.
     * @param inputs The inputs to update
     */
    default public void updateInputs(PivotInputs inputs) {}

    /**
     * Sets an output voltage to the pivot in volts (Range of -12 - 12 volts).
     * @param voltage The voltage to apply to the pivot motor.
     */
    default public void setOutputVoltage(double voltage) {}

    /**
     * Sets the target goal in rotations to the pivot using PID control.
     * @param targetPosition The target position to set the pivot in rotations.
     */
    default public void setTargetPosition(Rotation2d targetPosition) {}

    /**
     * Stops the pivot motor setting it to it's neutral mode output.
     */
    default public void stopPivotMotor() {}

    /**
     * Resets the encoder in the Pivot to zero rotations.
     */
    default public void resetPivotMotor() {}
}
