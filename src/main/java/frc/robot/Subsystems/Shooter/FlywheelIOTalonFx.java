package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Subsystems.Shooter.FlywheelIO.FlywheelInputs;
import frc.robot.Subsystems.Shooter.ShooterConstants.FlywheelGains;
import frc.robot.Subsystems.Shooter.ShooterConstants;

public class FlywheelIOTalonFx<HoodHardware> implements FlywheelIO {
    final TalonFX kFlywheelMotor;

    private TalonFXConfiguration kFlywheelConfig = new TalonFXConfiguration();

    private final StatusSignal<AngularVelocity> kFlywheelVelocity;
    private final StatusSignal<Voltage> kFlywheelVoltage;
    private final StatusSignal<Current> kFlywheelStatorCurrent;
    private final StatusSignal<Current> kFlywheelSupplyCurrent;

    private final VoltageOut kVoltageOut = new VoltageOut(0);
    private final VelocityVoltage kVelocityControl = new VelocityVoltage(0);
    private final PositionVoltage kPositionControl = new PositionVoltage(0);
    private final NeutralOut kNeutralOut = new NeutralOut();

    // Flywheel feedforward and feedback declarations
    private final PIDController flywheelPIDController;
    private SimpleMotorFeedforward flywheelFeedforward;
    private SlewRateLimiter flyhweelAccelerationLimiter;

    @AutoLogOutput(key = "Shooter/FlywheelVelocityGoalRotationsPerSec")
    private double kFlywheelVelocityGoalRotationsPerSec = 0.0;

    public FlywheelIOTalonFx(String canbus, HoodHardware hardware, FlywheelGains flywheelGains, double statusSignalUpdateFrequency) {
        kFlywheelMotor = new TalonFX(ShooterConstants.getInstance().kFlywheelMotorID, canbus);
        ShooterConstants constants = ShooterConstants.getInstance();

        // Flywheel signals
        kFlywheelVelocity = kFlywheelMotor.getVelocity();
        kFlywheelVoltage = kFlywheelMotor.getSupplyVoltage();
        kFlywheelStatorCurrent = kFlywheelMotor.getStatorCurrent();
        kFlywheelSupplyCurrent = kFlywheelMotor.getSupplyCurrent();

        // Flywheel feedforward and feedback initialization
        flywheelPIDController = new PIDController(
            flywheelGains.p(), 
            flywheelGains.i(),
            flywheelGains.d()
        );

        flywheelFeedforward = new SimpleMotorFeedforward(flywheelGains.s(), flywheelGains.v(), flywheelGains.a());
        flyhweelAccelerationLimiter = new SlewRateLimiter(flywheelGains.maxAccelerationRadiansPerSecondSquared());
    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {
        kFlywheelMotor.setVoltage(calculateFlywheelVolts(kFlywheelVelocityGoalRotationsPerSec));

        inputs.flywheelOk = BaseStatusSignal.refreshAll(
            kFlywheelVelocity,
            kFlywheelStatorCurrent,
            kFlywheelSupplyCurrent,
            kFlywheelVoltage
        ).isOK();

        inputs.flywheelVelocityRPM = kFlywheelVelocity.getValueAsDouble();
        inputs.flywheelVoltage = kFlywheelVoltage.getValueAsDouble();
        inputs.flywheelStatorCurrent = kFlywheelStatorCurrent.getValueAsDouble();
        inputs.flywheelSupplyCurrent = kFlywheelSupplyCurrent.getValueAsDouble();
    }

    public double calculateFlywheelVolts(double goalVelocity) {
        double PIDVolts = flywheelPIDController.calculate(kFlywheelMotor.getVelocity().getValueAsDouble(), goalVelocity);
        double feedforwardVolts = flywheelFeedforward.calculate(goalVelocity);

        return (PIDVolts + feedforwardVolts);
    }

     @Override
    public void setFlywheelVoltage(double volts) {
        kFlywheelMotor.setControl(kVoltageOut.withOutput(volts));
    }

    @Override
    public void setFlywheelVelocity(double rps) {
        kFlywheelMotor.setControl(kVelocityControl.withVelocity(rps).withSlot(0));
    }

    @Override
    public void stopFlywheel() {
        kFlywheelMotor.setControl(kNeutralOut);
    }
}
