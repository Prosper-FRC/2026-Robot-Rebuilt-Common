package frc.robot.Subsystems.Indexer;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.BaseStatusSignal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Subsystems.Indexer.IndexerConstantsMain.IndexerConstants;


public class IndexerTalonFX implements IndexerIO {

    private final TalonFX kHopperMotor1;
    private final TalonFX kHopperMotor2;
    private final TalonFX kHopperMotor3;
    private final TalonFX kIndexerMotor1;

    private final TalonFXConfiguration kHopperMotor1Config = new TalonFXConfiguration();
    private final TalonFXConfiguration kHopperMotor2Config = new TalonFXConfiguration();
    private final TalonFXConfiguration kHopperMotor3Config = new TalonFXConfiguration();
    private final TalonFXConfiguration kIndexerMotor1Config = new TalonFXConfiguration();

    /* Status Signals! */
    private final StatusSignal<AngularVelocity> kHopperMotor1VelocityRPS;
    private final StatusSignal<Voltage> kHopperMotor1Voltage;
    private final StatusSignal<Current> kHopperMotor1StatorCurrent;
    private final StatusSignal<Current> kHopperMotor1SupplyCurrent;

    private final StatusSignal<AngularVelocity> kHopperMotor2VelocityRPS;
    private final StatusSignal<Voltage> kHopperMotor2Voltage;
    private final StatusSignal<Current> kHopperMotor2StatorCurrent;
    private final StatusSignal<Current> kHopperMotor2SupplyCurrent;
    
    private final StatusSignal<AngularVelocity> kHopperMotor3VelocityRPS;
    private final StatusSignal<Voltage> kHopperMotor3Voltage;
    private final StatusSignal<Current> kHopperMotor3StatorCurrent;
    private final StatusSignal<Current> kHopperMotor3SupplyCurrent;

    private final StatusSignal<AngularVelocity> kIndexerMotor1VelocityRPS;
    private final StatusSignal<Voltage> kIndexerMotor1Voltage;
    private final StatusSignal<Current> kIndexerMotor1StatorCurrent;
    private final StatusSignal<Current> kIndexerMotor1SupplyCurrent;

    /* -------- Controls -------- */

    private final VoltageOut kVoltageOut = new VoltageOut(0);
    private final VelocityVoltage kVelocityControl = new VelocityVoltage(0);
    // private final PositionVoltage kPositionRotationsControl = new PositionVoltage(0);
    private final NeutralOut kNeutralOut = new NeutralOut();

    public IndexerTalonFX() {

        kHopperMotor1 = new TalonFX(IndexerConstants.kHopperMotor1ID);
        kHopperMotor2 = new TalonFX(IndexerConstants.kHopperMotor2ID);
        kHopperMotor3 = new TalonFX(IndexerConstants.kHopperMotor3ID);
        kIndexerMotor1 = new TalonFX(IndexerConstants.kIndexerMotor1ID);

        // Apply configs (PID, limits, inversion, etc.)
        kHopperMotor1.getConfigurator().apply(kHopperMotor1Config);
        kHopperMotor2.getConfigurator().apply(kHopperMotor2Config);
        kHopperMotor3.getConfigurator().apply(kHopperMotor3Config);
        kIndexerMotor1.getConfigurator().apply(kIndexerMotor1Config);
        
        // Hopper Motor 1 signals
        kHopperMotor1VelocityRPS = kHopperMotor1.getVelocity();
        kHopperMotor1Voltage = kHopperMotor1.getSupplyVoltage();
        kHopperMotor1StatorCurrent = kHopperMotor1.getStatorCurrent();
        kHopperMotor1SupplyCurrent = kHopperMotor1.getSupplyCurrent();

        // Hopper Motor 2 signals
        kHopperMotor2VelocityRPS = kHopperMotor2.getVelocity();
        kHopperMotor2Voltage = kHopperMotor2.getSupplyVoltage();
        kHopperMotor2StatorCurrent = kHopperMotor2.getStatorCurrent();
        kHopperMotor2SupplyCurrent = kHopperMotor2.getSupplyCurrent();

        // Hopper Motor 3 signals
        kHopperMotor3VelocityRPS = kHopperMotor3.getVelocity();
        kHopperMotor3Voltage = kHopperMotor3.getSupplyVoltage();
        kHopperMotor3StatorCurrent = kHopperMotor3.getStatorCurrent();
        kHopperMotor3SupplyCurrent = kHopperMotor3.getSupplyCurrent();

        // Indexer Motor 1 signals
        kIndexerMotor1VelocityRPS = kIndexerMotor1.getVelocity();
        kIndexerMotor1Voltage = kIndexerMotor1.getSupplyVoltage();
        kIndexerMotor1StatorCurrent = kIndexerMotor1.getStatorCurrent();
        kIndexerMotor1SupplyCurrent = kIndexerMotor1.getSupplyCurrent();
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {

        inputs.isIndexerMotor1Ok = BaseStatusSignal.refreshAll(
            kIndexerMotor1VelocityRPS,
            kIndexerMotor1Voltage,
            kIndexerMotor1StatorCurrent,
            kIndexerMotor1SupplyCurrent
        ).isOK();

        inputs.isHopperMotor1Ok = BaseStatusSignal.refreshAll(
            kHopperMotor1VelocityRPS,
            kHopperMotor1Voltage,
            kHopperMotor1StatorCurrent,
            kHopperMotor1SupplyCurrent
        ).isOK();

        inputs.isHopperMotor2Ok = BaseStatusSignal.refreshAll(
            kHopperMotor2VelocityRPS,
            kHopperMotor2Voltage,
            kHopperMotor2StatorCurrent,
            kHopperMotor2SupplyCurrent
        ).isOK();

        inputs.isHopperMotor3Ok = BaseStatusSignal.refreshAll(
            kHopperMotor3VelocityRPS,
            kHopperMotor3Voltage,
            kHopperMotor3StatorCurrent,
            kHopperMotor3SupplyCurrent
        ).isOK();

        inputs.kIndexMotor1VelocityRPS = kIndexerMotor1VelocityRPS.getValueAsDouble();
        inputs.kIndexerMotor1Voltage = kIndexerMotor1Voltage.getValueAsDouble();
        inputs.kIndexerMotor1StatorCurrent = kIndexerMotor1StatorCurrent.getValueAsDouble();
        inputs.kIndexerMotor1SupplyCurrent = kIndexerMotor1SupplyCurrent.getValueAsDouble();

        inputs.kHopperMotor1VelocityRPS = kHopperMotor1VelocityRPS.getValueAsDouble();
        inputs.kHopperMotor1Voltage = kHopperMotor1Voltage.getValueAsDouble();
        inputs.kHopperMotor1StatorCurrent = kHopperMotor1StatorCurrent.getValueAsDouble();
        inputs.kHopperMotor1SupplyCurrent = kHopperMotor1SupplyCurrent.getValueAsDouble();

        inputs.kHopperMotor2VelocityRPS = kHopperMotor2VelocityRPS.getValueAsDouble();
        inputs.kHopperMotor2Voltage = kHopperMotor2Voltage.getValueAsDouble();
        inputs.kHopperMotor2StatorCurrent = kHopperMotor2StatorCurrent.getValueAsDouble();
        inputs.kHopperMotor2SupplyCurrent = kHopperMotor2SupplyCurrent.getValueAsDouble();

        inputs.kHopperMotor3VelocityRPS = kHopperMotor3VelocityRPS.getValueAsDouble();
        inputs.kHopperMotor3Voltage = kHopperMotor3Voltage.getValueAsDouble();
        inputs.kHopperMotor3StatorCurrent = kHopperMotor3StatorCurrent.getValueAsDouble();
        inputs.kHopperMotor3SupplyCurrent = kHopperMotor3SupplyCurrent.getValueAsDouble();
    }

    // Motors voltage
    @Override
    public void setHopperMotor1Voltage(double voltage) {
        kHopperMotor1.setControl(kVoltageOut.withOutput(voltage));
    }

    @Override
    public void setHopperMotor2Voltage(double voltage) {
        kHopperMotor2.setControl(kVoltageOut.withOutput(voltage));
    }

    @Override
    public void setHopperMotor3Voltage(double voltage) {
        kHopperMotor3.setControl(kVoltageOut.withOutput(voltage));
    }

    @Override
    public void setIndexerMotor1Voltage(double voltage) {
        kIndexerMotor1.setControl(kVoltageOut.withOutput(voltage));
    }

    // Motor Velocity RPS
    @Override
    public void setMotorsVelocityRPS(double rps) {
        // Need to check over this!!
        kHopperMotor1.setControl(kVelocityControl.withVelocity(rps).withSlot(0));
        kHopperMotor2.setControl(kVelocityControl.withVelocity(rps).withSlot(1));
        kHopperMotor3.setControl(kVelocityControl.withVelocity(rps).withSlot(2));
        kIndexerMotor1.setControl(kVelocityControl.withVelocity(rps).withSlot(3));
    }

    // Stop the motors individually
    @Override
    public void stopHopperMotor1() {
        kHopperMotor1.setControl(kNeutralOut);
    }

    @Override
    public void stopHopperMotor2() {
        kHopperMotor2.setControl(kNeutralOut);
    }

    @Override
    public void stopHopperMotor3() {
        kHopperMotor3.setControl(kNeutralOut);
    }

    @Override
    public void stopIndexerMotor1() {
        kIndexerMotor1.setControl(kNeutralOut);
    }

    // Stop all the motors at once
    @Override
    public void stopMotors() {
        kHopperMotor1.setControl(kNeutralOut);
        kHopperMotor2.setControl(kNeutralOut);
        kHopperMotor3.setControl(kNeutralOut);
        kIndexerMotor1.setControl(kNeutralOut);
    }

    // Reset all the motor encoders individually - to do this we need to know what encoders we are using...
    // COME BACK TO THIS!
    @Override
    public void resetHopperMotor1() {
        kHopperMotor1.setPosition(0);
        kHopperMotor2.setPosition(0);
        kHopperMotor3.setPosition(0);
        kIndexerMotor1.setPosition(0);
    }
}
