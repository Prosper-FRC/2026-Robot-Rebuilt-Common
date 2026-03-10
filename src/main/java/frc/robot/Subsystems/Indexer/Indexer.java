package frc.robot.Subsystems.Indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class Indexer extends SubsystemBase {

    private final IndexerIO indexerIO;

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    public static enum IndexerState {
        // what states are a part of the indexer?
        // idle, active, eject --> we don't have more complex states
        Idle(() -> 0.0),
        Active(() -> 5.0), //dummy value
        Eject(() -> -5.00); //dummy value (RPS)

        private final DoubleSupplier goalVelocity;

        private IndexerState(DoubleSupplier goalVelocity) {
            this.goalVelocity = goalVelocity;
        }

        public double getGoalVelocity() {
            return goalVelocity.getAsDouble();
        }
    }

    // need the utils file for this
    private final IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

    @AutoLogOutput(key = "Indexer/State")
    public IndexerState state = IndexerState.Idle;

    public void setIndexerState(IndexerState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
    // identify what to do based on the states of the indexer!
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        switch (state) {
            case Idle:
                indexerIO.stopMotors();
                break;
            case Active:
                indexerIO.setMotorsVelocityRPS(state.getGoalVelocity());
                break;
            case Eject:
                indexerIO.setMotorsVelocityRPS(state.getGoalVelocity());
            default:
                break;
        }

        if (DriverStation.isDisabled()) {
            indexerIO.stopMotors();
        }
   }

// Below are all the methods identified earlier, not sure if we may need them later

   public void setIndexerMotor1Voltage(double volts) {
    indexerIO.setIndexerMotor1Voltage(volts);
   }


   public void stopIndexerMotor1() {
    indexerIO.stopIndexerMotor1();
   }

   public void setHopperMotor1Voltage(double volts) {
    indexerIO.setHopperMotor1Voltage(volts);
   }

   public void stopHopperMotor1Voltage(double volts) {
    indexerIO.stopHopperMotor1();
   }

   public void setHopperMotor2Voltage(double volts) {
    indexerIO.setHopperMotor2Voltage(volts);
   }

   public void stopHopperMotor2() {
    indexerIO.stopHopperMotor2();
   }

   public void setHopperMotor3Voltage(double volts) {
    indexerIO.setHopperMotor3Voltage(volts);
   }

   public void stopHopperMotor3() {
    indexerIO.stopHopperMotor3();
   }

   public double getIndexerMotor1VelocityRPS() {
    return inputs.kIndexMotor1VelocityRPS;
   }

   public double getHopperMotor1VelocityRPS() {
    return inputs.kHopperMotor1VelocityRPS;
   }

   public double getHopperMotor2VelocityRPS() {
    return inputs.kHopperMotor2VelocityRPS;
   }

   public double getHopperMotor3VelocityRPS() {
    return inputs.kHopperMotor3VelocityRPS;
   }

   public Command setIndexerStateCommand(IndexerState state) {
    return runOnce(() -> setIndexerState(state));
   }

   public Command setIndexerMotorsCommand(double rps) {
    return runOnce(() -> indexerIO.setMotorsVelocityRPS(rps));
   }
}