package frc.robot.Subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class ModuleTalonFX implements ModuleIO {
    private final TalonFX kDriveMotor;
    private final TalonFX kAzimuthMotor;
    private final TalonFXConfiguration kDriveConfiguration;
    private final TalonFXConfiguration kAzimuthConfiguration;
    private final CANcoder kCANcoder;

    private final moduleInputs kInputs = new moduleInputs();

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

    public ModuleTalonFX(int driveMotorID, int azimuthMotorID, int cancoderID, TalonFXConfiguration driveConfiguration, TalonFXConfiguration azimuthConfiguration) {
        kDriveMotor = new TalonFX(driveMotorID);
        kAzimuthMotor = new TalonFX(azimuthMotorID);
        kDriveConfiguration = driveConfiguration.clone();
        kAzimuthConfiguration = azimuthConfiguration.clone();
        kCANcoder = new CANcoder(cancoderID);

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
}
