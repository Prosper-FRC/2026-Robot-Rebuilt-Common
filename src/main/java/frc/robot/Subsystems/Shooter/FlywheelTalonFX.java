package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class FlywheelTalonFX implements FlywheelIO{
    private final TalonFX kFlywheelMotor;
    private final TalonFXConfiguration kFlywheelConfig;

    private final StatusSignal<AngularVelocity> kVelocity;
    private final StatusSignal<Voltage> kVoltage;
    private final StatusSignal<Current> kSupplyCurrent;
    private final StatusSignal<Current> kStatorCurrent;

    private final VelocityVoltage kVelocityControl = new VelocityVoltage(0.0d);
    private final VoltageOut kVoltageControl = new VoltageOut(0.0d);

    public FlywheelTalonFX() {
        kFlywheelMotor = new TalonFX(RobotConstants.ShooterConstants().flywheelID);
        kFlywheelConfig = new TalonFXConfiguration();
        
        var gains = RobotConstants.ShooterConstants().kShooterGains;

        kFlywheelConfig.Slot0.kP = gains.kP();
        kFlywheelConfig.Slot0.kI = gains.kI();
        kFlywheelConfig.Slot0.kD = gains.kD();
        kFlywheelConfig.Slot0.kV = gains.kV();

        kFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kFlywheelConfig.CurrentLimits.StatorCurrentLimit = RobotConstants.ShooterConstants().kFlywheelCurrentLimits.statorCurrent();
        kFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kFlywheelConfig.CurrentLimits.SupplyCurrentLimit = RobotConstants.ShooterConstants().kFlywheelCurrentLimits.supplyCurrent();

        kFlywheelMotor.getConfigurator().apply(kFlywheelConfig);
        
        kVelocity = kFlywheelMotor.getVelocity();
        kVoltage = kFlywheelMotor.getMotorVoltage();
        kStatorCurrent = kFlywheelMotor.getStatorCurrent();
        kSupplyCurrent = kFlywheelMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(flywheelInputs toUpdate) {
        toUpdate.isOk = BaseStatusSignal.refreshAll(
            kVelocity,
            kVoltage,
            kStatorCurrent,
            kSupplyCurrent
        ).isOK();

        toUpdate.velocityRPS = kVelocity.getValueAsDouble();
        toUpdate.appliedVoltage = kVoltage.getValueAsDouble();
        toUpdate.statorCurrent = kStatorCurrent.getValueAsDouble();
        toUpdate.supplyCurrent = kSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setFlywheelRPS(double rps) {
        kFlywheelMotor.setControl(kVelocityControl.withVelocity(rps).withSlot(0));
    }

    @Override
    public void setFlywheelVoltage(double voltage) {
        kFlywheelMotor.setControl(kVoltageControl.withOutput(voltage));
    }

    @Override
    public void stopFlywheel() {
        kFlywheelMotor.setControl(new NeutralOut());
    }
}
