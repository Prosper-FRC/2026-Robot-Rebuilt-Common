package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants;

public class PivotTalonFX implements PivotIO {
    private final TalonFX kPivotMotor;
    private final TalonFXConfiguration kPivotConfiguration;
    
    // Control modes
    private final VoltageOut kVoltageOut = new VoltageOut(0.0d);
    private final PositionVoltage kPositionControl = new PositionVoltage(0.0d);
    private final NeutralOut kNeutralOut = new NeutralOut();

    // Status Signals
    private final StatusSignal<Angle> kPosition;
    private final StatusSignal<AngularVelocity> kVelocity;
    private final StatusSignal<Temperature> kTemperature;
    private final StatusSignal<Voltage> kSupplyVoltage;
    private final StatusSignal<Current> kStatorCurrent;
    private final StatusSignal<Current> kSupplyCurrent; 

    // Constants ref
    private final IntakeConstants intakeConstants = RobotConstants.IntakeConstants();

    public PivotTalonFX() {
        kPivotMotor = new TalonFX(intakeConstants.kIntakeIDs.pivotID());
        kPivotConfiguration = new TalonFXConfiguration();
    
        // Configure motor
        kPivotConfiguration.Slot0.kP = intakeConstants.kIntakePivotGains.pidGains().kP();
        kPivotConfiguration.Slot0.kI = intakeConstants.kIntakePivotGains.pidGains().kI();
        kPivotConfiguration.Slot0.kD = intakeConstants.kIntakePivotGains.pidGains().kD();
        kPivotConfiguration.Slot0.kV = intakeConstants.kIntakePivotGains.feedForwardGains().kV();
        kPivotConfiguration.Slot0.kS = intakeConstants.kIntakePivotGains.feedForwardGains().kS();
        kPivotConfiguration.Slot0.kA = intakeConstants.kIntakePivotGains.feedForwardGains().kA();
        kPivotConfiguration.Slot0.kG = intakeConstants.kIntakePivotGains.feedForwardGains().kG();

        kPivotConfiguration.Voltage.PeakForwardVoltage = intakeConstants.kIntakeSoftLimits.voltageLimits();
        kPivotConfiguration.Voltage.PeakReverseVoltage = -intakeConstants.kIntakeSoftLimits.voltageLimits();
        kPivotConfiguration.Feedback.SensorToMechanismRatio = intakeConstants.kIntakeHardLimits.pivotGearRatio();
        kPivotConfiguration.MotorOutput.Inverted = intakeConstants.kIntakeSoftLimits.isInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        kPivotConfiguration.MotorOutput.NeutralMode = intakeConstants.kIntakeSoftLimits.isBraked() ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        // Apply configuration
        kPivotMotor.getConfigurator().apply(kPivotConfiguration);

        // Assign status signals
        kPosition = kPivotMotor.getPosition();
        kVelocity = kPivotMotor.getVelocity();
        kTemperature = kPivotMotor.getDeviceTemp();
        kSupplyVoltage = kPivotMotor.getSupplyVoltage();
        kStatorCurrent = kPivotMotor.getStatorCurrent();
        kSupplyCurrent = kPivotMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(PivotInputs toUpdate) {
        toUpdate.isOk = BaseStatusSignal.refreshAll(
            kPosition,
            kVelocity,
            kTemperature,
            kSupplyVoltage,
            kStatorCurrent,
            kSupplyCurrent).isOK();

        toUpdate.positionRotations = kPosition.getValueAsDouble();
        toUpdate.velocityRPS = kVelocity.getValueAsDouble();
        toUpdate.temperatureCelcius = kTemperature.getValueAsDouble();
        toUpdate.supplyVoltage = kSupplyVoltage.getValueAsDouble();
        toUpdate.statorCurrent = kStatorCurrent.getValueAsDouble();
        toUpdate.supplyCurrent = kSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setOutputVoltage(double voltage) {
        kPivotMotor.setControl(kVoltageOut.withOutput(voltage));
    }

    @Override
    public void setTargetPosition(Rotation2d targetPosition) {
        kPivotMotor.setControl(kPositionControl.withPosition(targetPosition.getRotations()).withSlot(0));
    }

    @Override
    public void stopPivotMotor() {
        kPivotMotor.setControl(kNeutralOut);
    }

    @Override
    public void resetPivotMotor() {
        kPivotMotor.setPosition(0.0d);
    }
}
