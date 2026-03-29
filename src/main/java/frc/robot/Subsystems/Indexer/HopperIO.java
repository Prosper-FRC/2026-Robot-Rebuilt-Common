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

    default public void updateInputs(hopperInputs toUpdate) {}

    default public void setHopperVoltage(double volts) {}

    default public void stopHopper() {}
}
