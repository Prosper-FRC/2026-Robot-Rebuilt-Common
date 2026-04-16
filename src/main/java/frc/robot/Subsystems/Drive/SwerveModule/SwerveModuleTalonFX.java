package frc.robot.Subsystems.Drive.SwerveModule;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants.SwerveModuleGains;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants.SwerveModuleIds;

public class SwerveModuleTalonFX implements SwerveModuleIO {
    // Declare motors
    public final TalonFX kDriveMotor;
    public final TalonFX kAzimuthMotor;
    public final CANcoder kCancoder;
    
    public final TalonFXConfiguration kDriveMotorConfiguration;
    public final TalonFXConfiguration kAzimuthMotorConfiguration;
    public final CANcoderConfiguration kCancoderConfiguration;

    // Declare status signals
    public final StatusSignal<Voltage> kDriveSupplyVoltage;
    public final StatusSignal<Voltage> kDriveMotorVoltage;
    public final StatusSignal<Current> kDriveSupplyCurrent;
    public final StatusSignal<Current> kDriveStatorCurrent;
    public final StatusSignal<Angle> kDrivePosition;
    public final StatusSignal<AngularVelocity> kDriveVelocity;
    public final StatusSignal<AngularAcceleration> kDriveAcceleration;

    public final StatusSignal<Voltage> kAzimuthSupplyVoltage;
    public final StatusSignal<Voltage> kAzimuthMotorVoltage;
    public final StatusSignal<Current> kAzimuthSupplyCurrent;
    public final StatusSignal<Current> kAzimuthStatorCurrent;
    public final StatusSignal<Angle> kAzimuthPosition;
    public final StatusSignal<AngularVelocity> kAzimuthVelocity;
    public final StatusSignal<AngularAcceleration> kAzimuthAcceleration;

    public final StatusSignal<Angle> kCancoderPosition;
    public final StatusSignal<AngularVelocity> kCancoderVelocity;

    // Declare control signals
    public final VoltageOut kDriveVoltageOutControl = new VoltageOut(0.0d);
    public final VelocityVoltage kDriveVelocityVoltageControl = new VelocityVoltage(0.0d);
    public final VelocityTorqueCurrentFOC kDriveVelocityTorqueCurrentControl = new VelocityTorqueCurrentFOC(0.0d);
    public final NeutralOut kDriveNeutralOut = new NeutralOut();

    public final VoltageOut kAzimuthVoltageOutControl = new VoltageOut(0.0d);
    public final PositionVoltage kAzimuthPositionVoltageControl = new PositionVoltage(0.0d);
    public final PositionTorqueCurrentFOC kAzimuthPositionTorqueCurrentControl = new PositionTorqueCurrentFOC(0.0d);
    public final NeutralOut kAzimuthNeutralOut = new NeutralOut();

    public final double azimuthOffset;

    public SwerveModuleTalonFX(SwerveModuleIds moduleIds, double moduleOffset, CANBus canbus) {
        kDriveMotor = new TalonFX(moduleIds.driveId(), canbus);
        kAzimuthMotor = new TalonFX(moduleIds.azimuthId(), canbus);
        kCancoder = new CANcoder(moduleIds.cancoderId(), canbus);

        kDriveMotorConfiguration = new TalonFXConfiguration();
        kAzimuthMotorConfiguration = new TalonFXConfiguration();
        kCancoderConfiguration = new CANcoderConfiguration();

        // Configure motors
        SwerveModuleGains driveGains = RobotConstants.DriveConstants().kDriveGains;

        kDriveMotorConfiguration
            .withSlot0(
                new Slot0Configs()
                    .withKP(driveGains.kP())
                    .withKI(driveGains.kI())
                    .withKD(driveGains.kD())
                    .withKV(driveGains.kV())
                    .withKS(driveGains.kS())
                    .withKA(driveGains.kA())
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(RobotConstants.DriveConstants().kModuleCurrentLimits.driveStatorLimit())
                    .withSupplyCurrentLimit(RobotConstants.DriveConstants().kModuleCurrentLimits.driveSupplyLimit())
            )
            // .withMotionMagic(
            //     new MotionMagicConfigs()
            //         .withMotionMagicCruiseVelocity(RobotConstants.DriveConstants().kMotionMagicGains.drive_cruise_velocity())
            //         .withMotionMagicAcceleration(RobotConstants.DriveConstants().kMotionMagicGains.drive_acceleration())
            // )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(RobotConstants.DriveConstants().kDriveOutputConfigs.isCCWPositive() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
                    .withNeutralMode(RobotConstants.DriveConstants().kDriveOutputConfigs.isBrakedNeutral() ? NeutralModeValue.Brake : NeutralModeValue.Coast)
            )
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(RobotConstants.DriveConstants().kModuleHardware.driveGearReduction())
            );

        SwerveModuleGains azimuthGains = RobotConstants.DriveConstants().kAzimuthGains;

        kAzimuthMotorConfiguration
            .withSlot0(
                new Slot0Configs()
                    .withKP(azimuthGains.kP())
                    .withKI(azimuthGains.kI())
                    .withKD(azimuthGains.kD())
                    .withKV(azimuthGains.kV())
                    .withKS(azimuthGains.kS())
                    .withKA(azimuthGains.kA())
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(RobotConstants.DriveConstants().kModuleCurrentLimits.azimuthStatorLimit())
                    .withSupplyCurrentLimit(RobotConstants.DriveConstants().kModuleCurrentLimits.azimuthSupplyLimit())
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(RobotConstants.DriveConstants().kAzimuthOutputConfigs.isCCWPositive() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
                    .withNeutralMode(RobotConstants.DriveConstants().kAzimuthOutputConfigs.isBrakedNeutral() ? NeutralModeValue.Brake : NeutralModeValue.Coast)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(RobotConstants.DriveConstants().kModuleHardware.azimuthGearReduction())
            )
            .withClosedLoopGeneral(
                new ClosedLoopGeneralConfigs()
                    .withContinuousWrap(true)
            );
        
        // Apply motor configurations
        kDriveMotor.getConfigurator().apply(kDriveMotorConfiguration);
        kAzimuthMotor.getConfigurator().apply(kAzimuthMotorConfiguration);
        kCancoder.getConfigurator().apply(kCancoderConfiguration);
        
        // Set status signals
        kDriveSupplyVoltage = kDriveMotor.getSupplyVoltage();
        kDriveMotorVoltage = kDriveMotor.getMotorVoltage();
        kDriveSupplyCurrent = kDriveMotor.getSupplyCurrent();
        kDriveStatorCurrent = kDriveMotor.getStatorCurrent();
        kDrivePosition = kDriveMotor.getPosition();
        kDriveVelocity = kDriveMotor.getVelocity();
        kDriveAcceleration = kDriveMotor.getAcceleration();

        kAzimuthSupplyVoltage = kAzimuthMotor.getSupplyVoltage();
        kAzimuthMotorVoltage = kAzimuthMotor.getMotorVoltage();
        kAzimuthSupplyCurrent = kAzimuthMotor.getSupplyCurrent();
        kAzimuthStatorCurrent = kAzimuthMotor.getStatorCurrent();
        kAzimuthPosition = kAzimuthMotor.getPosition();
        kAzimuthVelocity = kAzimuthMotor.getVelocity();
        kAzimuthAcceleration = kAzimuthMotor.getAcceleration();

        kCancoderPosition = kCancoder.getAbsolutePosition();
        kCancoderVelocity = kCancoder.getVelocity();

        azimuthOffset = moduleOffset;

        // Optimize Canbus
        BaseStatusSignal.setUpdateFrequencyForAll(
            250,
            kDriveSupplyVoltage,
            kDriveMotorVoltage,
            kDriveSupplyCurrent,
            kDriveStatorCurrent,
            kDrivePosition,
            kDriveVelocity,
            kDriveAcceleration,

            kAzimuthSupplyVoltage,
            kAzimuthMotorVoltage,
            kAzimuthSupplyCurrent,
            kAzimuthStatorCurrent,
            kAzimuthPosition,
            kAzimuthVelocity,
            kAzimuthAcceleration,

            kCancoderPosition,
            kCancoderVelocity
        );

        kDriveMotor.optimizeBusUtilization();
        kAzimuthMotor.optimizeBusUtilization();
        kCancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.driveModuleOk = BaseStatusSignal.refreshAll(
            kDriveSupplyVoltage,
            kDriveMotorVoltage,
            kDriveSupplyCurrent,
            kDriveStatorCurrent,
            kDrivePosition,
            kDriveVelocity,
            kDriveAcceleration
        ).isOK();
        inputs.driveModuleSupplyVoltage = kDriveSupplyVoltage.getValueAsDouble();
        inputs.driveModuleMotorVoltage = kDriveMotorVoltage.getValueAsDouble();
        inputs.driveModuleSupplyCurrent = kDriveSupplyCurrent.getValueAsDouble();
        inputs.driveModuleStatorCurrent = kDriveStatorCurrent.getValueAsDouble();
        inputs.driveModulePositionRotations = BaseStatusSignal.getLatencyCompensatedValueAsDouble(kDrivePosition, kDriveVelocity);
        inputs.driveModuleSpeedRotationsPerSecond = kDriveVelocity.getValueAsDouble();
        inputs.driveModuleAccelerationRotationsPerSecondSquared = kDriveAcceleration.getValueAsDouble();

        inputs.azimuthModuleOk = BaseStatusSignal.refreshAll(
            kAzimuthSupplyVoltage,
            kAzimuthMotorVoltage,
            kAzimuthSupplyCurrent,
            kAzimuthStatorCurrent,
            kAzimuthPosition,
            kAzimuthVelocity,
            kAzimuthAcceleration
        ).isOK();
        inputs.azimuthModuleSupplyVoltage = kAzimuthSupplyVoltage.getValueAsDouble();
        inputs.azimuthModuleMotorVoltage = kAzimuthMotorVoltage.getValueAsDouble();
        inputs.azimuthModuleSupplyCurrent = kAzimuthSupplyCurrent.getValueAsDouble();
        inputs.azimuthModuleStatorCurrent = kAzimuthStatorCurrent.getValueAsDouble();
        inputs.azimuthModulePositionRotations = BaseStatusSignal.getLatencyCompensatedValueAsDouble(kAzimuthPosition, kAzimuthVelocity);
        inputs.azimuthModuleSpeedRotationsPerSecond = kAzimuthVelocity.getValueAsDouble();
        inputs.azimuthModuleAccelerationRotationsPerSecondSquared = kAzimuthAcceleration.getValueAsDouble();

        inputs.cancoderOk = BaseStatusSignal.refreshAll(
            kCancoderPosition,
            kCancoderVelocity
        ).isOK();

        inputs.cancoderPositionRotations = StatusSignal.getLatencyCompensatedValueAsDouble(kCancoderPosition, kCancoderVelocity);
        inputs.cancoderOffsetPositionRotations = inputs.cancoderPositionRotations - azimuthOffset;
        inputs.cancoderSpeedRotationsPerSecond = kCancoderVelocity.getValueAsDouble();
    }

    @Override
    public void setDriveSpeedWithVoltage(double rotationsPerSecond) {
        kDriveMotor.setControl(kDriveVelocityVoltageControl.withVelocity(rotationsPerSecond).withSlot(0));
    }

    @Override
    public void setDriveSpeedWithFOC(double rotationsPerSecond) {
        kDriveMotor.setControl(kDriveVelocityTorqueCurrentControl.withVelocity(rotationsPerSecond).withSlot(0));
    }

    @Override
    public void setDriveMotorVoltage(double voltage) {
        kDriveMotor.setControl(kDriveVoltageOutControl.withOutput(voltage));
    }

    @Override
    public void stopDriveMotor() {
        kDriveMotor.setControl(kDriveNeutralOut);
    }

    @Override
    public void setAzimuthPositionWithVoltage(double rotations) {
        kAzimuthMotor.setControl(kAzimuthPositionVoltageControl.withPosition(rotations).withSlot(0));
    }

    @Override
    public void setAzimuthPositionWithFOC(double rotations) {
        kAzimuthMotor.setControl(kAzimuthPositionTorqueCurrentControl.withPosition(rotations).withSlot(0));
    }

    @Override
    public void setAzimuthMotorVoltage(double voltage) {
        kAzimuthMotor.setControl(kAzimuthVoltageOutControl.withOutput(voltage));
    }

    @Override
    public void stopAzimuthMotor() {
        kAzimuthMotor.setControl(kAzimuthNeutralOut);
    }

    @Override
    public void seedAbsoluteEncoder() {
        double absolute_position = BaseStatusSignal.getLatencyCompensatedValueAsDouble(kCancoderPosition, kCancoderVelocity) - azimuthOffset;
        kAzimuthMotor.setPosition(absolute_position);
    }
}
