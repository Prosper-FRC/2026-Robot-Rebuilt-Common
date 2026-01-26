package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Subsystems.Drive.DriveConstants.moduleIDs;
import frc.robot.Subsystems.Drive.DriveConstants.moduleOffsets;

public class ModuleTalonFX implements ModuleIO {
    private final TalonFX kDriveMotor;
    private final TalonFX kAzimuthMotor;
    private final TalonFXConfiguration kDriveConfiguration;
    private final TalonFXConfiguration kAzimuthConfiguration;
    private final CANcoder kCANcoder;
    private final CANcoderConfiguration kCANCoderConfig;

    // Status Signals for logging and stuff.
    private final StatusSignal<Angle> kDriveRotations;
    private final StatusSignal<AngularVelocity> kDriveRPS;
    private final StatusSignal<Temperature> kDriveTemperature;
    private final StatusSignal<Current> kDriveStatorCurrent;
    private final StatusSignal<Current> kDriveSupplyCurrent;
    private final StatusSignal<Voltage> kDriveSupplyVoltage;

    private final StatusSignal<Angle> kAzimuthRotations;
    private final StatusSignal<AngularVelocity> kAzimuthRPS;
    private final StatusSignal<Temperature> kAzimuthTemperature;
    private final StatusSignal<Current> kAzimuthStatorCurrent;
    private final StatusSignal<Current> kAzimuthSupplyCurrent;
    private final StatusSignal<Voltage> kAzimuthSupplyVoltage;

    // TODO: Set up FOC once we have the subscription.
    private final VelocityVoltage kVelocityControl = new VelocityVoltage(0.0d);
    private final PositionVoltage kPositionControl = new PositionVoltage(0.0d);
    private final VoltageOut kVoltageControl = new VoltageOut(0.0d);
    private final NeutralOut kNeutralControl = new NeutralOut();

    public ModuleTalonFX(moduleIDs ids, moduleOffsets offsets) {
        kDriveConfiguration = new TalonFXConfiguration();
        kAzimuthConfiguration = new TalonFXConfiguration();
        kCANCoderConfig = new CANcoderConfiguration();

        kDriveMotor = new TalonFX(ids.driveID());
        kAzimuthMotor = new TalonFX(ids.azimuthID());
        kCANcoder = new CANcoder(ids.canCoderID());

        // Configure motors
        kDriveConfiguration.CurrentLimits.StatorCurrentLimit = DriveConstants.getInstance().kDriveCurrentConfig.statorCurrentLimit();
        kDriveConfiguration.CurrentLimits.SupplyCurrentLimit = DriveConstants.getInstance().kDriveCurrentConfig.supplyCurrentLimit();
        kDriveConfiguration.CurrentLimits.StatorCurrentLimitEnable = DriveConstants.getInstance().kDriveCurrentConfig.useStatorCurrentLimit();
        kDriveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.getInstance().kDriveCurrentConfig.useSupplyCurrentLimit();
        kDriveConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.getInstance().kDriveCurrentConfig.maxForwardTorque();
        kDriveConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = DriveConstants.getInstance().kDriveCurrentConfig.maxReverseTorque();
        kDriveConfiguration.Feedback.SensorToMechanismRatio = DriveConstants.getInstance().kDriveGearing;
        kDriveConfiguration.MotorOutput.Inverted = DriveConstants.getInstance().kDriveMotorOutputConfig.isInverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        kDriveConfiguration.MotorOutput.NeutralMode = DriveConstants.getInstance().kDriveMotorOutputConfig.isBraked() ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        kDriveConfiguration.MotorOutput.PeakForwardDutyCycle = DriveConstants.getInstance().kDriveMotorOutputConfig.peakForwardDutyCycle();
        kDriveConfiguration.MotorOutput.PeakForwardDutyCycle = DriveConstants.getInstance().kDriveMotorOutputConfig.peakReverseDutyCycle();
        kDriveConfiguration.Voltage.PeakForwardVoltage = DriveConstants.getInstance().kDriveVoltageConfig.peakForwardVoltage();
        kDriveConfiguration.Voltage.PeakReverseVoltage = DriveConstants.getInstance().kDriveVoltageConfig.peakReverseVoltage();

        kAzimuthConfiguration.CurrentLimits.StatorCurrentLimit = DriveConstants.getInstance().kAzimuthCurrentConfig.statorCurrentLimit();
        kAzimuthConfiguration.CurrentLimits.SupplyCurrentLimit = DriveConstants.getInstance().kAzimuthCurrentConfig.supplyCurrentLimit();
        kAzimuthConfiguration.CurrentLimits.StatorCurrentLimitEnable = DriveConstants.getInstance().kAzimuthCurrentConfig.useStatorCurrentLimit();
        kAzimuthConfiguration.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.getInstance().kAzimuthCurrentConfig.useSupplyCurrentLimit();
        kAzimuthConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.getInstance().kAzimuthCurrentConfig.maxForwardTorque();
        kAzimuthConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = DriveConstants.getInstance().kAzimuthCurrentConfig.maxReverseTorque();
        kAzimuthConfiguration.Feedback.SensorToMechanismRatio = DriveConstants.getInstance().kAzimuthGearing;
        kAzimuthConfiguration.MotorOutput.Inverted = DriveConstants.getInstance().kAzimuthMotorOutputConfig.isInverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        kAzimuthConfiguration.MotorOutput.NeutralMode = DriveConstants.getInstance().kAzimuthMotorOutputConfig.isBraked() ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        kAzimuthConfiguration.MotorOutput.PeakForwardDutyCycle = DriveConstants.getInstance().kAzimuthMotorOutputConfig.peakForwardDutyCycle();
        kAzimuthConfiguration.MotorOutput.PeakForwardDutyCycle = DriveConstants.getInstance().kAzimuthMotorOutputConfig.peakReverseDutyCycle();
        kAzimuthConfiguration.Voltage.PeakForwardVoltage = DriveConstants.getInstance().kAzimuthVoltageConfig.peakForwardVoltage();
        kAzimuthConfiguration.Voltage.PeakReverseVoltage = DriveConstants.getInstance().kAzimuthVoltageConfig.peakReverseVoltage();
        kCANCoderConfig.MagnetSensor.MagnetOffset = offsets.rotationOffset();
        kAzimuthConfiguration.Feedback.FeedbackRemoteSensorID = kCANcoder.getDeviceID();
        kAzimuthConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        // Setting PIDF gains.
        kDriveConfiguration.Slot0.kP = DriveConstants.getInstance().kDrivePIDController.getP();
        kDriveConfiguration.Slot0.kI = DriveConstants.getInstance().kDrivePIDController.getI();
        kDriveConfiguration.Slot0.kD = DriveConstants.getInstance().kDrivePIDController.getD();
        kDriveConfiguration.Slot0.kA = DriveConstants.getInstance().kDriveGains.kA();
        kDriveConfiguration.Slot0.kS = DriveConstants.getInstance().kDriveGains.kS();
        kDriveConfiguration.Slot0.kV = DriveConstants.getInstance().kDriveGains.kV();

        kAzimuthConfiguration.Slot0.kP = DriveConstants.getInstance().kAzimuthPIDController.getP();
        kAzimuthConfiguration.Slot0.kI = DriveConstants.getInstance().kAzimuthPIDController.getI();
        kAzimuthConfiguration.Slot0.kD = DriveConstants.getInstance().kAzimuthPIDController.getD();
        kAzimuthConfiguration.Slot0.kA = DriveConstants.getInstance().kAzimuthGains.kA();
        kAzimuthConfiguration.Slot0.kS = DriveConstants.getInstance().kAzimuthGains.kS();
        kAzimuthConfiguration.Slot0.kV = DriveConstants.getInstance().kAzimuthGains.kV();
        // TODO: Motion magic!!!

        // Apply the configurations
        kDriveMotor.getConfigurator().apply(kDriveConfiguration);
        kAzimuthMotor.getConfigurator().apply(kAzimuthConfiguration);
        kCANcoder.getConfigurator().apply(kCANCoderConfig);

        // Assigning status signals
        kDriveRotations = kDriveMotor.getPosition();
        kDriveRPS = kDriveMotor.getVelocity();
        kDriveTemperature = kDriveMotor.getDeviceTemp();
        kDriveStatorCurrent = kDriveMotor.getStatorCurrent();
        kDriveSupplyCurrent = kDriveMotor.getSupplyCurrent();
        kDriveSupplyVoltage = kDriveMotor.getSupplyVoltage();

        kAzimuthRotations = kCANcoder.getAbsolutePosition();
        kAzimuthRPS = kCANcoder.getVelocity();
        kAzimuthTemperature = kAzimuthMotor.getDeviceTemp();
        kAzimuthStatorCurrent = kAzimuthMotor.getStatorCurrent();
        kAzimuthSupplyCurrent = kAzimuthMotor.getSupplyCurrent();
        kAzimuthSupplyVoltage = kAzimuthMotor.getSupplyVoltage();
    }

    @Override
    public void updateInputs(moduleInputs toUpdate) {
        toUpdate.driveOk = BaseStatusSignal.refreshAll(
            kDriveRotations,
            kDriveRPS,
            kDriveTemperature,
            kDriveStatorCurrent,
            kDriveSupplyCurrent,
            kDriveSupplyVoltage)
        .isOK();

        toUpdate.azimuthOk = BaseStatusSignal.refreshAll(
            kAzimuthTemperature,
            kAzimuthStatorCurrent,
            kAzimuthSupplyCurrent,
            kAzimuthSupplyVoltage)
        .isOK();

        toUpdate.CANCoderOk = BaseStatusSignal.refreshAll(kAzimuthRotations, kAzimuthRPS).isOK();

        toUpdate.drivePositionRotations = kDriveRotations.getValueAsDouble();
        toUpdate.driveVelocityRPS = kDriveRPS.getValueAsDouble();
        toUpdate.driveTemperatureCelcius = kDriveTemperature.getValueAsDouble();
        toUpdate.driveStatorCurrent = kDriveStatorCurrent.getValueAsDouble();
        toUpdate.driveSupplyCurrent = kDriveSupplyCurrent.getValueAsDouble();
        toUpdate.driveSupplyVoltage = kDriveSupplyVoltage.getValueAsDouble();

        toUpdate.azimuthPositionRotations = kAzimuthRotations.getValueAsDouble();
        toUpdate.azimuthVelocityRPS = kAzimuthRPS.getValueAsDouble();
        toUpdate.azimuthTemperatureCelcius = kAzimuthTemperature.getValueAsDouble();
        toUpdate.azimuthStatorCurrent = kAzimuthStatorCurrent.getValueAsDouble();
        toUpdate.azimuthSupplyCurrent = kAzimuthSupplyCurrent.getValueAsDouble();
        toUpdate.azimuthSupplyVoltage = kAzimuthSupplyVoltage.getValueAsDouble();
    }

    // Drive methods
    @Override
    public void setDriveRotations(double rotations) {
        kDriveMotor.setControl(kPositionControl.withPosition(rotations).withSlot(0));
    }

    @Override
    public void setDriveRPS(double rps) {
        kDriveMotor.setControl(kVelocityControl.withVelocity(rps).withSlot(0));
    }
    
    @Override
    public void setDriveVoltage(double volts) {
        kDriveMotor.setControl(kVoltageControl.withOutput(volts));
    }

    @Override
    public void stopDrive() {
        kDriveMotor.setControl(kNeutralControl);
    }

    @Override
    public void resetDrive() {
        kDriveMotor.setPosition(0.0d);
    }

    // Azimuth methods
    @Override
    public void setAzimuthRotations(double rotations) {
        kAzimuthMotor.setControl(kPositionControl.withPosition(rotations).withSlot(0));
    }

    @Override
    public void setAzimuthRPS(double rps) {
        kAzimuthMotor.setControl(kVelocityControl.withVelocity(rps).withSlot(0));
    }

    @Override
    public void setAzimuthVoltage(double volts) {
        kAzimuthMotor.setControl(kVoltageControl.withOutput(volts));
    }

    @Override
    public void stopAzimuth() {
        kAzimuthMotor.setControl(kNeutralControl);
    }

    @Override
    public void resetAzimuth() {
        kCANcoder.setPosition(0.0d);
    }

    @Override
    public void updateDrivePIDValues(double kP, double kI, double kD) {}

    @Override
    public void updateAzimuthPIDValues(double kP, double kI, double kD) {}

}
