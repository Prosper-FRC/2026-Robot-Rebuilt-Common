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
     * Sets the goal position of both indexer motors in rotations.
     * @param volts The position in rotations for the motor to target.
     */
    
    /**
     * Sets the goal velocity of both indexer motors in rps.
     * @param rps The rotations per second to target.
     */
    default public void setMotorVelocityRPS(double rps) {}

    /**
     * Sets the voltage applied to the first indexer motor in volts.
     * @param voltage The voltage to apply to the motor.
     */
    default public void setIndexerMotor1Voltage(double voltage) {}

    default public void setHopperMotor1Voltage(double voltage) {}

    default public void setHopperMotor2Voltage(double voltage) {}

    default public void setHopperMotor3Voltage(double voltage) {}
    /**
     * Sets the voltage applied to the second indexer motor in volts.
     * @param voltage The voltage to apply to the motor.
     */

    /**
     * Resets the relative encoder of the first indexer motor to 0.
     */
    default public void resetHopperMotor1() {}

    /**
     * Resets the relative encoder of the second indexer motor to 0.
     */
    default public void resetHopperMotor2() {}

    /**
     * Resets both relative encoders in both of the indexer motors to 0.
     */
    default public void resetHopperMotor3() {}

    default public void resetIndexerMotor1() {}

    default public void resetMotor() {}
    /**
     * Sets the first indexer motor to its current neutral mode.
     */
    default public void stopHopperMotor1() {}

    /**
     * Sets the second indexer motor to its current neutral mode.
     */
    default public void stopHopperMotor2() {}

    default public void stopHopperMotor3() {}

    default public void stopIndexerMotor1() {}
    /**
     * Sets both indexer motors to their current neutral modes.
     */
    default public void stopMotor() {}
}
