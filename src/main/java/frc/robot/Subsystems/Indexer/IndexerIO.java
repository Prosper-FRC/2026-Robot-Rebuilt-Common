package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    static class IndexerInputs {
        public boolean isIndexerMotor1Ok = false;
        public double indexMotorPositionRotations1 = 0.0d;
        public double indexMotorVelocityRPS1 = 0.0d;
        public double voltageIndexer1 = 0.0d;
        public double statorCurrentIndexer1 = 0.0d;
        public double supplyCurrentIndexer1 = 0.0d;

        public boolean isIndexerMotor2Ok = false;
        public double indexMotorPositionRotations2 = 0.0d;
        public double indexMotorVelocityRPS2 = 0.0d;
        public double voltageIndexer2 = 0.0d;
        public double statorCurrentIndexer2 = 0.0d;
        public double supplyCurrentIndexer2 = 0.0d;
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
    default public void setMotorsPositionsRotations(double rotations) {}

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

    /**
     * Sets the voltage applied to the second indexer motor in volts.
     * @param voltage The voltage to apply to the motor.
     */
    default public void setIndexerMotor2Voltage(double voltage) {}

    /**
     * Resets the relative encoder of the first indexer motor to 0.
     */
    default public void resetMotor1() {}

    /**
     * Resets the relative encoder of the second indexer motor to 0.
     */
    default public void resetMotor2() {}

    /**
     * Resets both relative encoders in both of the indexer motors to 0.
     */
    default public void resetMotors() {}

    /**
     * Sets the first indexer motor to its current neutral mode.
     */
    default public void stopMotor1() {}

    /**
     * Sets the second indexer motor to its current neutral mode.
     */
    default public void stopMotor2() {}

    /**
     * Sets both indexer motors to their current neutral modes.
     */
    default public void stopMotor() {}
}
