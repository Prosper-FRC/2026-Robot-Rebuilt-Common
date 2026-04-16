package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public class hopperInputs {
        public boolean isOk = false;
        public double hopperRPS = 0.0d;
        public double hopperVoltage = 0.0d;
        public double hopperStatorCurrent = 0.0d;
        public double hopperSupplyCurrent = 0.0d;
    }

    /**
     * Updates NT logging data from given inputs.
     * @param inputs The inputs to update
     */
    default public void updateInputs(hopperInputs inputs) {}

    /**
     * Sets the voltage supplied to the hopper (Range of -12 - 12 volts).
     * @param volts The number of volts to supply to the hopper.
     */
    default public void setHopperVoltage(double volts) {}

    default public void stopHopper() {}
}