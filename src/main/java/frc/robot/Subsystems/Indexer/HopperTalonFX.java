package frc.robot.Subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HopperTalonFX implements HopperIO {
    public final TalonFX kHopperMotor;

    public final StatusSignal<AngularVelocity> kHopperVelocity;
    public final StatusSignal<Voltage> kHopperAppliedVoltage;
    public final StatusSignal<Current> kHopperStatorCurrent;
    public final StatusSignal<Current> kHopperSupplyCurrent;

    public final VoltageOut kVoltageControl = new VoltageOut(0.0d);
    
    public HopperTalonFX(int hopperId) {
        kHopperMotor = new TalonFX(hopperId);

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
        kHopperMotor.setControl(kVoltageControl);
    }

    @Override
    public void stopHopper() {
        kHopperMotor.setControl(new NeutralOut());
    }


}
