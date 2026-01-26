package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Subsystems.Drive.DriveConstants.moduleIDs;

public class ModuleTalonFX implements ModuleIO {
    private final TalonFX kDriveMotor;
    private final TalonFX kAzimuthMotor;
    private final TalonFXConfiguration kDriveConfiguration;
    private final TalonFXConfiguration kAzimuthConfiguration;
    private final CANcoder kCANcoder;

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

    private final VelocityVoltage kVelocityControl = new VelocityVoltage(0.0d);
    private final PositionVoltage kPositionControl = new PositionVoltage(0.0d);
    private final VoltageOut kVoltageControl = new VoltageOut(0.0d);
    private final NeutralOut kNeutralControl = new NeutralOut();

    public ModuleTalonFX(moduleIDs ids) {
        kDriveConfiguration = new TalonFXConfiguration();
        kAzimuthConfiguration = new TalonFXConfiguration();

        kDriveMotor = new TalonFX(ids.driveID());
        kAzimuthMotor = new TalonFX(ids.azimuthID());
        kCANcoder = new CANcoder(ids.canCoderID());

        kDriveMotor.getConfigurator().apply(kDriveConfiguration);
        kAzimuthMotor.getConfigurator().apply(kAzimuthConfiguration);

        // Assigning status signals
        kDriveRotations = kDriveMotor.getPosition();
        kDriveRPS = kDriveMotor.getVelocity();
        kDriveTemperature = kDriveMotor.getDeviceTemp();
        kDriveStatorCurrent = kDriveMotor.getStatorCurrent();
        kDriveSupplyCurrent = kDriveMotor.getSupplyCurrent();
        kDriveSupplyVoltage = kDriveMotor.getSupplyVoltage();

        kAzimuthRotations = kAzimuthMotor.getPosition();
        kAzimuthRPS = kAzimuthMotor.getVelocity();
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
            kAzimuthRotations,
            kAzimuthRPS,
            kAzimuthTemperature,
            kAzimuthStatorCurrent,
            kAzimuthSupplyCurrent,
            kAzimuthSupplyVoltage)
        .isOK();

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
    public void resetAzimuth(double position) {
        kCANcoder.setPosition(0.0d);
    }

    @Override
    public void updateDrivePIDValues(double kP, double kI, double kD) {}

    @Override
    public void updateAzimuthPIDValues(double kP, double kI, double kD) {}

}
