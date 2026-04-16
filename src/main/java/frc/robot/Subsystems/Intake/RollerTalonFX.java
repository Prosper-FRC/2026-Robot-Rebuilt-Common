package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

public class RollerTalonFX implements RollerIO {
    private final TalonFX kRollerMotor;
    private final TalonFXConfiguration kRollerConfiguration;
    
    // Control modes
    private final VoltageOut kVoltageOut = new VoltageOut(0.0d);
    private final VelocityVoltage kVelocityControl = new VelocityVoltage(0.0d);
    private final NeutralOut kNeutralOut = new NeutralOut();

    // Status Signals
    private final StatusSignal<Angle> kPosition;
    private final StatusSignal<AngularVelocity> kVelocity;
    private final StatusSignal<Temperature> kTemperature;
    private final StatusSignal<Voltage> kSupplyVoltage;
    private final StatusSignal<Current> kStatorCurrent;
    private final StatusSignal<Current> kSupplyCurrent; 

    // Constants ref
    private IntakeConstants intakeConstants = RobotConstants.IntakeConstants();

    public RollerTalonFX() {
        kRollerMotor = new TalonFX(intakeConstants.kIntakeIDs.rollerID());
        kRollerConfiguration = new TalonFXConfiguration();
    
        // Configure motor
        kRollerConfiguration.Feedback.SensorToMechanismRatio = intakeConstants.kIntakeHardLimits.rollerGearRatio();
        kRollerConfiguration.MotorOutput.Inverted = intakeConstants.kIntakeSoftLimits.isInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        kRollerConfiguration.MotorOutput.NeutralMode = intakeConstants.kIntakeSoftLimits.isBraked() ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        kRollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        kRollerConfiguration.CurrentLimits.SupplyCurrentLimit = RobotConstants.IntakeConstants().rollerCurrentLimits.supplyCurrent();
        kRollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        kRollerConfiguration.CurrentLimits.StatorCurrentLimit = RobotConstants.IntakeConstants().rollerCurrentLimits.statorCurrent();

        // Apply configuration
        kRollerMotor.getConfigurator().apply(kRollerConfiguration);

        // Assign status signals
        kPosition = kRollerMotor.getPosition();
        kVelocity = kRollerMotor.getVelocity();
        kTemperature = kRollerMotor.getDeviceTemp();
        kSupplyVoltage = kRollerMotor.getMotorVoltage();
        kStatorCurrent = kRollerMotor.getStatorCurrent();
        kSupplyCurrent = kRollerMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(RollerInputs inputs) {
        inputs.isOk = BaseStatusSignal.refreshAll(
            kPosition,
            kVelocity,
            kTemperature,
            kSupplyVoltage,
            kStatorCurrent,
            kSupplyCurrent).isOK();

        inputs.positionRotations = kPosition.getValueAsDouble();
        inputs.velocityRPS = kVelocity.getValueAsDouble();
        inputs.temperatureCelcius = kTemperature.getValueAsDouble();
        inputs.supplyVoltage = kSupplyVoltage.getValueAsDouble();
        inputs.statorCurrent = kStatorCurrent.getValueAsDouble();
        inputs.supplyCurrent = kSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setOutputVoltage(double voltage) {
        kRollerMotor.setControl(kVoltageOut.withOutput(voltage));
    }

    @Override
    public void stopRollerMotor() {
        kRollerMotor.setControl(kNeutralOut);
    }
}
