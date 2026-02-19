package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    static class IntakeInputs {
        public boolean isPivotOk = false;
        public double pivotPositionRotations = 0.0d;
        public double pivotVelocityRPS = 0.0d;
        public double pivotTemperatureCelcius = 0.0d;
        public double pivotVoltage = 0.0d;
        public double pivotStatorCurrent = 0.0d;
        public double pivotSupplyCurrent = 0.0d;

        public boolean isRollerOk = false;
        public double rollerPositionRotations = 0.0d;
        public double rollerVelocityRPS = 0.0d;
        public double rollerTemperatureCelcius = 0.0d;
        public double rollerVoltage = 0.0d;
        public double rollerStatorCurrent = 0.0d;
        public double rollerSupplyCurrent = 0.0d;
    }

    /**
     * Updates the given inputs to the most recent values.
     * @param toUpdate The inputs to update.
     */
    default public void updateInputs(IntakeInputs toUpdate) {}
    
    /**
     * Sets the voltage output of the pivot motor.
     * @param volts The voltage to output.
     */
    default public void setPivotVoltageOut(double volts) {}

    /**
     * Sets the number of rotations in the absolute encoder for the pivot to target.
     * @param rotations The rotations to target.
     */
    default public void setPivotPositionRotations(double rotations) {}

    /**
     * Stops the pivot in place (Not reliable for holding the pivot).
     */
    default public void stopPivot() {}

    /**
     * Resets the pivot's absolute encoder.
     */
    default public void resetPivotEncoder() {}

    /**
     * Sets the voltage output of the roller motor.
     * @param volts The voltage to output.
     */
    default public void setRollerVoltageOut(double volts) {}

    /**
     * Sets the number of rotations per second for the motor to target.
     * @param rps The number of rotations per second to target
     */
    default public void setRollerSpeedRPS(double rps) {}

    /**
     * Stops the roller in place.
     */
    default public void stopRoller() {}

    /**
     * Resets the roller's relative encode.
     */
    default public void resetRollerEncoder() {}
}
