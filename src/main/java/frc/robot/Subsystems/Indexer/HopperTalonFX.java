package frc.robot.Subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class HopperTalonFX implements HopperIO {
    public final TalonFX kHopperMotor;
    public final TalonFXConfiguration kHopperConfig;

    public final StatusSignal<AngularVelocity> kHopperVelocity;
    public final StatusSignal<Voltage> kHopperAppliedVoltage;
    public final StatusSignal<Current> kHopperStatorCurrent;
    public final StatusSignal<Current> kHopperSupplyCurrent;

    public final VoltageOut kVoltageControl = new VoltageOut(0.0d);
    
    public HopperTalonFX(int hopperId, boolean shouldInvert) {
        kHopperMotor = new TalonFX(hopperId);
        kHopperConfig = new TalonFXConfiguration();
        kHopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kHopperConfig.CurrentLimits.StatorCurrentLimit = RobotConstants.IndexerConstants().kStatorCurrentLimit;
        kHopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kHopperConfig.CurrentLimits.SupplyCurrentLimit = RobotConstants.IndexerConstants().kSupplyCurrentLimit;
        kHopperConfig.MotorOutput.Inverted = shouldInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        kHopperMotor.getConfigurator().apply(kHopperConfig);

        kHopperVelocity = kHopperMotor.getVelocity();
        kHopperAppliedVoltage = kHopperMotor.getMotorVoltage();
        kHopperStatorCurrent = kHopperMotor.getStatorCurrent();
        kHopperSupplyCurrent = kHopperMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(hopperInputs toUpdate) {
        toUpdate.isOk = BaseStatusSignal.refreshAll(
            kHopperVelocity,
            kHopperAppliedVoltage,
            kHopperStatorCurrent,
            kHopperSupplyCurrent
        ).isOK();

        toUpdate.hopperRPS = kHopperVelocity.getValueAsDouble();
        toUpdate.hopperVoltage = kHopperAppliedVoltage.getValueAsDouble();
        toUpdate.hopperStatorCurrent = kHopperStatorCurrent.getValueAsDouble();
        toUpdate.hopperSupplyCurrent = kHopperSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setHopperVoltage(double volts) {
        kHopperMotor.setControl(kVoltageControl.withOutput(volts));
    }

    @Override
    public void stopHopper() {
        kHopperMotor.setControl(new NeutralOut());
    }
}
