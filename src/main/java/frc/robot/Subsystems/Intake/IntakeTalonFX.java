package frc.robot.Subsystems.Intake;

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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants.IntakeIDs;

public class IntakeTalonFX implements IntakeIO {
    private final TalonFX kPivotMotor;
    private final TalonFX kRollerMotor;
    private final CANcoder kPivotCancoder;
    private final TalonFXConfiguration kPivotConfig;
    private final TalonFXConfiguration kRollerConfig;
    private final CANcoderConfiguration kCancoderConfiguration;

    private final StatusSignal<Angle> kPivotAngle;
    private final StatusSignal<AngularVelocity> kPivotVelocity;
    private final StatusSignal<Temperature> kPivotTemperature;
    private final StatusSignal<Voltage> kPivotVoltage;
    private final StatusSignal<Current> kPivotStatorCurrent;
    private final StatusSignal<Current> kPivotSupplyCurrent;
    private final StatusSignal<Angle> kRollerAngle;
    private final StatusSignal<AngularVelocity> kRollerVelocity;
    private final StatusSignal<Temperature> kRollerTemperature;
    private final StatusSignal<Voltage> kRollerVoltage;
    private final StatusSignal<Current> kRollerStatorCurrent;
    private final StatusSignal<Current> kRollerSupplyCurrent;

    private final VoltageOut kVoltageControl = new VoltageOut(0.0d);
    private final PositionVoltage kPositionControl = new PositionVoltage(0.0d);
    private final VelocityVoltage kVelocityControl = new VelocityVoltage(0.0d);
    private final NeutralOut kNeutralControl = new NeutralOut();

    public IntakeTalonFX(IntakeIDs ids) {
        kPivotMotor = new TalonFX(ids.PivotID());
        kRollerMotor = new TalonFX(ids.RollerID());
        kPivotCancoder = new CANcoder(ids.CANCoderID());

        kPivotConfig = new TalonFXConfiguration();
        kRollerConfig = new TalonFXConfiguration();
        kCancoderConfiguration = new CANcoderConfiguration();
        System.out.println("Nikhil was here");

        // Configure the motors and the CANCoder here from the records in constants.

        kPivotMotor.getConfigurator().apply(kPivotConfig);
        kRollerMotor.getConfigurator().apply(kRollerConfig);
        kPivotCancoder.getConfigurator().apply(kCancoderConfiguration);

        kPivotAngle = kPivotMotor.getPosition();
        kPivotVelocity = kPivotMotor.getVelocity();
        kPivotTemperature = kPivotMotor.getDeviceTemp();
        kPivotVoltage = kPivotMotor.getSupplyVoltage();
        kPivotStatorCurrent = kPivotMotor.getStatorCurrent();
        kPivotSupplyCurrent = kPivotMotor.getSupplyCurrent();
        kRollerAngle = kRollerMotor.getPosition();
        kRollerVelocity = kRollerMotor.getVelocity();
        kRollerTemperature = kRollerMotor.getDeviceTemp();
        kRollerVoltage = kRollerMotor.getSupplyVoltage();
        kRollerStatorCurrent = kRollerMotor.getStatorCurrent();
        kRollerSupplyCurrent = kRollerMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(IntakeInputs toUpdate) {
        toUpdate.isPivotOk = BaseStatusSignal.refreshAll(
            kPivotAngle,
            kPivotVelocity,
            kPivotTemperature,
            kPivotVoltage,
            kPivotStatorCurrent,
            kPivotSupplyCurrent
        ).isOK();

        toUpdate.isRollerOk = BaseStatusSignal.refreshAll(
            kRollerAngle,
            kRollerVelocity,
            kRollerTemperature,
            kRollerVoltage,
            kRollerStatorCurrent,
            kRollerSupplyCurrent
        ).isOK();

        toUpdate.pivotPositionRotations = kPivotAngle.getValueAsDouble();
        toUpdate.pivotVelocityRPS = kPivotVelocity.getValueAsDouble();
        toUpdate.pivotTemperatureCelcius = kPivotTemperature.getValueAsDouble();
        toUpdate.pivotVoltage = kPivotVoltage.getValueAsDouble();
        toUpdate.pivotStatorCurrent = kPivotStatorCurrent.getValueAsDouble();
        toUpdate.pivotSupplyCurrent = kPivotSupplyCurrent.getValueAsDouble();
        
        toUpdate.rollerPositionRotations = kRollerAngle.getValueAsDouble();
        toUpdate.rollerVelocityRPS = kRollerVelocity.getValueAsDouble();
        toUpdate.rollerTemperatureCelcius = kRollerTemperature.getValueAsDouble();
        toUpdate.rollerVoltage = kRollerVoltage.getValueAsDouble();
        toUpdate.rollerStatorCurrent = kRollerStatorCurrent.getValueAsDouble();
        toUpdate.rollerSupplyCurrent = kRollerSupplyCurrent.getValueAsDouble();

    }

    @Override
    public void setPivotVoltageOut(double volts) {
        kPivotMotor.setControl(kVoltageControl.withOutput(volts));
    }

    @Override
    public void setPivotPositionRotations(double rotations) {
        kPivotMotor.setControl(kPositionControl.withPosition(rotations).withSlot(0));
    }

    @Override
    public void stopPivot() {
        kPivotMotor.setControl(kNeutralControl);
    }

    @Override
    public void resetPivotEncoder() {
        kPivotMotor.setPosition(0.0d);
    }

    @Override
    public void setRollerVoltageOut(double volts) {
        kRollerMotor.setControl(kVoltageControl.withOutput(volts));
    }

    @Override
    public void setRollerSpeedRPS(double rps) {
        kRollerMotor.setControl(kVelocityControl.withVelocity(rps).withSlot(0));
    }

    @Override
    public void stopRoller() {
        kRollerMotor.setControl(kNeutralControl);
    }

    @Override
    public void resetRollerEncoder() {
        kPivotCancoder.setPosition(0.0d);
    }
}
