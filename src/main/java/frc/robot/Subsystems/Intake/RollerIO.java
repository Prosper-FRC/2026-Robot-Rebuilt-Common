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

    /**
     * Updates NT logging data from given inputs.
     * @param inputs The inputs to update
     */
    default public void updateInputs(RollerInputs inputs) {}

    /**
     * Sets the output voltage to the roller motor in volts (Range of -12 - 12 volts).
     * @param voltage The voltage to apply to the roller.
     */
    default public void setOutputVoltage(double voltage) {}

    /**
     * Stops the roller motor using its neutral mode value.
     */
    default public void stopRollerMotor() {}
}
