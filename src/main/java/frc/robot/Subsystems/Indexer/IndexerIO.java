package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    static class IndexerInputs {
        // need to change the names to match those of IndexerTalonFX - 3 hopper motors instead of 2 indexer
        public boolean isIndexerMotor1Ok = false;
        // public double indexMotorPositionRotations1 = 0.0d;
        public double kIndexMotor1VelocityRPS = 0.0d;
        public double kIndexerMotor1Voltage = 0.0d;
        public double kIndexerMotor1StatorCurrent = 0.0d;
        public double kIndexerMotor1SupplyCurrent = 0.0d;

        public boolean isHopperMotor1Ok = false;
        // public double hopperMotor1Position = 0.0d;
        public double kHopperMotor1VelocityRPS = 0.0d;
        public double kHopperMotor1Voltage = 0.0d;
        public double kHopperMotor1StatorCurrent = 0.0d;
        public double kHopperMotor1SupplyCurrent = 0.0d;

         public boolean isHopperMotor2Ok = false;
        // public double hopperMotor1Position = 0.0d;
        public double kHopperMotor2VelocityRPS = 0.0d;
        public double kHopperMotor2Voltage = 0.0d;
        public double kHopperMotor2StatorCurrent = 0.0d;
        public double kHopperMotor2SupplyCurrent = 0.0d;

         public boolean isHopperMotor3Ok = false;
        // public double hopperMotor1Position = 0.0d;
        public double kHopperMotor3VelocityRPS = 0.0d;
        public double kHopperMotor3Voltage = 0.0d;
        public double kHopperMotor3StatorCurrent = 0.0d;
        public double kHopperMotor3SupplyCurrent = 0.0d;
    }

    /**
     * Updates the given inputs to the most recent values.
     * @param toUpdate The inputs to update.
     */
    
    default public void updateInputs(IndexerInputs toUpdate) {}
    
    /**
     * Sets the goal velocity of all of the motors in rps.
     * @param rps The rotations per second to target.
     */
    default public void setMotorsVelocityRPS(double rps) {}

    /**
     * Sets the voltage applied to the each motor in volts.
     * @param voltage The voltage to apply to the motor.
     */
    default public void setIndexerMotor1Voltage(double voltage) {}

    default public void setHopperMotor1Voltage(double voltage) {}

    default public void setHopperMotor2Voltage(double voltage) {}

    default public void setHopperMotor3Voltage(double voltage) {}

    /**
     * Resets the relative encoder of the motors to 0.
     */
    default public void resetHopperMotor1() {}

    default public void resetHopperMotor2() {}

    default public void resetHopperMotor3() {}

    default public void resetIndexerMotor1() {}


    default public void resetMotor() {}
    /**
     * Sets the each motor to its current neutral mode.
     */
    default public void stopHopperMotor1() {}

    default public void stopHopperMotor2() {}

    default public void stopHopperMotor3() {}

    default public void stopIndexerMotor1() {}
    /**
     * Sets all the motors to their current neutral modes at once.
     */
    default public void stopMotors() {}
}
