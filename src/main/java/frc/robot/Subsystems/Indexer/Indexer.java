package frc.robot.Subsystems.Indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.measure.Voltage;

// utils is currently not on this branch
import frc.robot.utils.debugging.LoggedTunableNumber;

import com.ctre.phoenix6.hardware.TalonFX;

public class Indexer extends SubsystemBase {

    // enum - define the states to apply the state machine here & based on the states - (if, else) define what should happen
    
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final VoltageOut setVoltage = new VoltageOut(0);

   @Override
   public void periodic() {
    // identify what to do based on the states of the indexer!

    

    indexerIO.updateInputs(inputs);
   }

   public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
   }

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
    indexerIO.stopHopperMotor1();
   }

   public void setHopperMotor3Voltage(double volts) {
    indexerIO.setHopperMotor3Voltage(volts);
   }

   public void stopHopperMotor3() {
    indexerIO.stopHopperMotor3();
   }

   public double getIndexerMotor1VelocityRPS() {
    return inputs.kIndexerMotor1VelocityRPS;
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
}