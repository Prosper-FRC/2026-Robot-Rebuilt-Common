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
    
    /**
     * Updates NT logging data from given inputs.
     * @param inputs The inputs to update
     */
    default public void updateInputs(flywheelInputs inputs) {}

    /**
     * Sets the flywheel's speed in rotations per second (At freespin it can hit 90rps at absolute max).
     * @param rps The rotations per second to set the flywheel to.
     */
    default public void setFlywheelRPS(double rps) {}

    /**
     * Sets the flywheel's motor voltage (Range of -12 - 12 volts).
     * @param voltage The voltage to set the flywheel to
     */
    default public void setFlywheelVoltage(double voltage) {}

    /**
     * Stops the flywheel and sets it to it's neutral mode.
     */
    default public void stopFlywheel() {}
}
