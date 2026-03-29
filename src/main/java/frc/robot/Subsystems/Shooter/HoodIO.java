package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public class hoodInputs {
        public boolean isOk = false;
        public double hoodPositionRotation = 0.0d;
        public double hoodVelocityRPS = 0.0d;
        public double voltage = 0.0d;
        public double statorCurrent = 0.0d;
        public double supplyCurrent = 0.0d;
    }
    
    default public void updateInputs(hoodInputs toUpdate) {}

    default public void setHoodPosition(double rotations) {}
    
    default public void setHoodVoltage(double volts) {}

    default public void stopHood() {}
}
