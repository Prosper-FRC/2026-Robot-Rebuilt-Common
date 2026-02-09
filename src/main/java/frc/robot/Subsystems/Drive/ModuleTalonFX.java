package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants.moduleGains;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants.moduleIDs;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants.moduleOffsets;

public class ModuleTalonFX implements ModuleIO {
    // Create objects to hold motor and cancoder references.
    private final TalonFX kDrive;
    private final TalonFX kAzimuth;
    private final CANcoder kCANcoder;

    // Create configuration instances to fully refresh motor configurations.
    private final TalonFXConfiguration kDriveConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration kAzimuthConfiguration = new TalonFXConfiguration();
    private final CANcoderConfiguration kCANcoderConfiguration = new CANcoderConfiguration();

    // Create StatusSignal objects to hold motor/cancoder status.
    private final StatusSignal<Angle> kDrivePosition;
    private final StatusSignal<AngularVelocity> kDriveVelocity;
    private final StatusSignal<Temperature> kDriveTemperature;
    private final StatusSignal<Current> kDriveStatorCurrent;
    private final StatusSignal<Current> kDriveSupplyCurrent;
    private final StatusSignal<Voltage> kDriveSupplyVoltage;

    private final StatusSignal<Angle> kAzimuthPosition;
    private final StatusSignal<AngularVelocity> kAzimuthVelocity;
    private final StatusSignal<Temperature> kAzimuthTemperature;
    private final StatusSignal<Current> kAzimuthStatorCurrent;
    private final StatusSignal<Current> kAzimuthSupplyCurrent;
    private final StatusSignal<Voltage> kAzimuthSupplyVoltage;
    
    private final StatusSignal<Angle> kCANcoderPosition;

    // Create control objects to apply a control to the motors.
    private final MotionMagicVoltage kPositionControl = new MotionMagicVoltage(0.0d);
    private final MotionMagicVelocityVoltage kVelocityControl = new MotionMagicVelocityVoltage(0.0d);
    private final VoltageOut kVoltageControl = new VoltageOut(0.0d);
    private final NeutralOut kNeutralControl = new NeutralOut();

    public ModuleTalonFX(moduleIDs ids, moduleOffsets offsets, moduleGains gains, String CANBus) {
        kDrive = new TalonFX(ids.driveID(), CANBus);
        kAzimuth = new TalonFX(ids.azimuthID(), CANBus);
        kCANcoder = new CANcoder(ids.CANcoderID(), CANBus);
        
        // TODO: Reconfigure the motors with premium feature once we've activated the licenses
        ///// DRIVE MOTOR /////
        // Drive Gains
        kDriveConfiguration.Slot0.kP = gains.driveGains().kP();
        kDriveConfiguration.Slot0.kI = gains.driveGains().kI();
        kDriveConfiguration.Slot0.kD = gains.driveGains().kD();
        kDriveConfiguration.Slot0.kS = gains.driveGains().kS();
        kDriveConfiguration.Slot0.kV = gains.driveGains().kV();
        kDriveConfiguration.Slot0.kA = gains.driveGains().kA();
        kDriveConfiguration.MotionMagic.MotionMagicAcceleration =  RobotConstants.DriveConstants().kModuleGains.driveMMGains().maxAcceleration();

        // Drive Configurations
        kDriveConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        kDriveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        kDriveConfiguration.CurrentLimits.StatorCurrentLimit = RobotConstants.DriveConstants().kModuleCurrentLimits.driveStatorCurrentLimit();
        kDriveConfiguration.CurrentLimits.SupplyCurrentLimit = RobotConstants.DriveConstants().kModuleCurrentLimits.driveSupplyCurrentLimit();
        kDriveConfiguration.Voltage.PeakForwardVoltage = RobotConstants.DriveConstants().kModuleVoltageLimits.driveVoltagePeakRange();
        kDriveConfiguration.Voltage.PeakReverseVoltage = -RobotConstants.DriveConstants().kModuleVoltageLimits.driveVoltagePeakRange();
        kDriveConfiguration.Feedback.SensorToMechanismRatio = RobotConstants.DriveConstants().kModuleHardLimits.driveGearRatio();

        ///// AZIMUTH MOTOR /////
        // Azimuth Gains
        kAzimuthConfiguration.Slot0.kP = gains.azimuthGains().kP();
        kAzimuthConfiguration.Slot0.kI = gains.azimuthGains().kI();
        kAzimuthConfiguration.Slot0.kD = gains.azimuthGains().kD();
        kAzimuthConfiguration.Slot0.kS = gains.azimuthGains().kS();
        kAzimuthConfiguration.Slot0.kV = gains.azimuthGains().kV();
        kAzimuthConfiguration.Slot0.kA = gains.azimuthGains().kA();

        // Azimuth Configurations
        kAzimuthConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        kAzimuthConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        kAzimuthConfiguration.CurrentLimits.StatorCurrentLimit = RobotConstants.DriveConstants().kModuleCurrentLimits.azimuthStatorCurrentLimit();
        kAzimuthConfiguration.CurrentLimits.SupplyCurrentLimit = RobotConstants.DriveConstants().kModuleCurrentLimits.azimuthSupplyCurrentLimit();
        kAzimuthConfiguration.Voltage.PeakForwardVoltage = RobotConstants.DriveConstants().kModuleVoltageLimits.azimuthVoltagePeakRange();
        kAzimuthConfiguration.Voltage.PeakReverseVoltage = -RobotConstants.DriveConstants().kModuleVoltageLimits.azimuthVoltagePeakRange();
        kAzimuthConfiguration.Feedback.SensorToMechanismRatio = RobotConstants.DriveConstants().kModuleHardLimits.azimuthGearRatio();

        // Extra Azimuth Configuration
        kAzimuthConfiguration.Feedback.FeedbackRemoteSensorID = kCANcoder.getDeviceID();
        kAzimuthConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        kAzimuthConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

        ///// CANCODER /////
        kCANcoderConfiguration.MagnetSensor.MagnetOffset = offsets.rotationalOffset().getRotations();

        // Applying the configurations
        kDrive.getConfigurator().apply(kDriveConfiguration);
        kAzimuth.getConfigurator().apply(kAzimuthConfiguration);
        kCANcoder.getConfigurator().apply(kCANcoderConfiguration);

        // Getting the StatusSignals from the motors and cancoder.
        kDrivePosition = kDrive.getPosition();
        kDriveVelocity = kDrive.getVelocity();
        kDriveTemperature = kDrive.getDeviceTemp();
        kDriveStatorCurrent = kDrive.getStatorCurrent();
        kDriveSupplyCurrent = kDrive.getSupplyCurrent();
        kDriveSupplyVoltage = kDrive.getMotorVoltage();

        kAzimuthPosition = kAzimuth.getPosition();
        kAzimuthVelocity = kAzimuth.getVelocity();
        kAzimuthTemperature = kAzimuth.getDeviceTemp();
        kAzimuthStatorCurrent = kAzimuth.getStatorCurrent();
        kAzimuthSupplyCurrent = kAzimuth.getSupplyCurrent();
        kAzimuthSupplyVoltage = kAzimuth.getMotorVoltage();

        kCANcoderPosition = kCANcoder.getAbsolutePosition();
    }

    @Override
    public void updateInputs(moduleInputs toUpdate) {
        // Refresh all StatusSignals for latest data.
        toUpdate.driveOk = BaseStatusSignal.refreshAll(
            kDrivePosition,
            kDriveVelocity,
            kDriveTemperature,
            kDriveStatorCurrent,
            kDriveSupplyCurrent,
            kDriveSupplyVoltage
        ).isOK();
        
        toUpdate.azimuthOk = BaseStatusSignal.refreshAll(
            kAzimuthPosition,
            kAzimuthVelocity,
            kAzimuthTemperature,
            kAzimuthStatorCurrent,
            kAzimuthSupplyCurrent,
            kAzimuthSupplyVoltage
        ).isOK();

        toUpdate.CANCoderOk = BaseStatusSignal.refreshAll(
            kCANcoderPosition
        ).isOK();

        // Set the driveInput values to the refreshed status signals.
        toUpdate.drivePositionRotations = kDrivePosition.getValueAsDouble();
        toUpdate.driveVelocityRPS = kDriveVelocity.getValueAsDouble();
        toUpdate.driveTemperatureCelcius = kDriveTemperature.getValueAsDouble();
        toUpdate.driveStatorCurrent = kDriveStatorCurrent.getValueAsDouble();
        toUpdate.driveSupplyCurrent = kDriveSupplyCurrent.getValueAsDouble();
        toUpdate.driveSupplyVoltage = kDriveSupplyVoltage.getValueAsDouble();

        toUpdate.azimuthPositionRotations = kAzimuthPosition.getValueAsDouble();
        toUpdate.azimuthVelocityRPS = kAzimuthVelocity.getValueAsDouble();
        toUpdate.azimuthTemperatureCelcius = kAzimuthTemperature.getValueAsDouble();
        toUpdate.azimuthStatorCurrent = kAzimuthStatorCurrent.getValueAsDouble();
        toUpdate.azimuthSupplyCurrent = kAzimuthSupplyCurrent.getValueAsDouble();
        toUpdate.azimuthSupplyVoltage = kAzimuthSupplyVoltage.getValueAsDouble();

        toUpdate.CANCoderPositionRotations = kCANcoderPosition.getValueAsDouble();
    }

    // Drive specific methods
    @Override
    public void setDriveRotations(double rotations) {
        kDrive.setControl(kPositionControl.withPosition(rotations).withSlot(0));
    }

    @Override
    public void setDriveRPS(double rps) {
        kDrive.setControl(kVelocityControl.withVelocity(rps).withSlot(0));
    }
    
    @Override
    public void setDriveVoltage(double volts) {
        kDrive.setControl(kVoltageControl.withOutput(volts));
    }

    @Override
    public void stopDrive() {
        kDrive.setControl(kNeutralControl);
    }

    @Override
    public void resetDrive() {
        kDrive.setPosition(0.0d);
    }

    // Azimuth specific methods
    @Override
    public void setAzimuthRotations(double rotations) {
        kAzimuth.setControl(kPositionControl.withPosition(rotations).withSlot(0));
    }

    @Override
    public void setAzimuthRPS(double rps) {
        kAzimuth.setControl(kVelocityControl.withVelocity(rps).withSlot(0));
    }
    
    @Override
    public void setAzimuthVoltage(double volts) {
        kAzimuth.setControl(kVoltageControl.withOutput(volts));
    }

    @Override
    public void stopAzimuth() {
        kAzimuth.setControl(kNeutralControl);
    }

    @Override
    public void resetAzimuth() {
        kAzimuth.setPosition(0.0d);
    }

    // CANcoder specific methods
    @Override
    public void resetCANcoder() {
        kCANcoder.setPosition(0.0d);
    }

    // Misc
    @Override
    public void updateDrivePIDValues(double kP, double kI, double kD) {
        kDriveConfiguration.Slot0.kP = kP;
        kDriveConfiguration.Slot0.kI = kI;
        kDriveConfiguration.Slot0.kD = kD;

        kDrive.getConfigurator().apply(kDriveConfiguration);
    }

    @Override
    public void updateAzimuthPIDValues(double kP, double kI, double kD) {
        kAzimuthConfiguration.Slot0.kP = kP;
        kAzimuthConfiguration.Slot0.kI = kI;
        kAzimuthConfiguration.Slot0.kD = kD;

        kAzimuth.getConfigurator().apply(kAzimuthConfiguration);
    }
}
