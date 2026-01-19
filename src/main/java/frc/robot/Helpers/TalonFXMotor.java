package frc.robot.Helpers;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

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

/**
 * This class should almost always run through a subsystem layer for extra subsystem specific logging/features as well as command based methods.
 */
public class TalonFXMotor {
    @AutoLog
    static class motorInputs {
        public boolean isOk = false;
        public double relativePositionRotations = 0.0d;
        public double velocityRotationsPerSecond = 0.0d;
        public double temperatureCelcius = 0.0d;
        public double statorCurrent = 0.0d;
        public double supplyCurrent = 0.0d;
        public double supplyVoltage = 0.0d;
    }

    private final TalonFXConfiguration kConfig;
    private final TalonFX kMotorController;
    // Using an optional for type safety since people will be writing wrappers for this class.
    private final Optional<CANcoder> kAbsoluteEncoder;

    private final VoltageOut kVoltageOut = new VoltageOut(0.0d);
    private final VelocityVoltage kVelocityOut = new VelocityVoltage(0.0d);
    private final PositionVoltage kPositionOut = new PositionVoltage(0.0d);

    private final StatusSignal<Angle> kPosition;
    private final StatusSignal<AngularVelocity> kVelocity;
    private final StatusSignal<Temperature> kTemperature;
    private final StatusSignal<Current> kStatorCurrent;
    private final StatusSignal<Current> kSupplyCurrent;
    private final StatusSignal<Voltage> kVoltage;

    /**
     * This is a generic motor class for a TalonFX motor, the class combines the IO layer and hardware layer together for
     * ease of use, and to account for the fact we will not be using any other motor controller than a TalonFX controller.
     * @param id The ID of the motor
     * @param configuration The configuration of the motor (which can usually be referenced through the respective subsystem constants.) this is a shallow copy.
     */
    public TalonFXMotor(int id, TalonFXConfiguration configuration) {
        kAbsoluteEncoder = Optional.empty();
        kConfig = configuration;
        kMotorController = new TalonFX(id);
        kMotorController.getConfigurator().apply(kConfig);

        kPosition = kMotorController.getPosition();
        kVelocity = kMotorController.getVelocity();
        kTemperature = kMotorController.getDeviceTemp();
        kStatorCurrent = kMotorController.getStatorCurrent();
        kSupplyCurrent = kMotorController.getSupplyCurrent();
        kVoltage = kMotorController.getMotorVoltage();
    }
    /**
     * This is a generic motor class for a TalonFX motor, the class combines the IO layer and hardware layer together for
     * ease of use, and to account for the fact we will not be using any other motor controller than a TalonFX controller.
     * @param id The ID of the motor
     * @param CANcoderID The id of the CANcoder (Absolute encoder) that this device runs on.
     * @param configuration The configuration of the motor (which can usually be referenced through the respective subsystem constants.)
     */
    public TalonFXMotor(int id, int CANcoderID, TalonFXConfiguration configuration) {
        kAbsoluteEncoder = Optional.of(new CANcoder(CANcoderID));
        kConfig = configuration.clone();
        kMotorController = new TalonFX(id);
        kMotorController.getConfigurator().apply(kConfig);

        kPosition = kAbsoluteEncoder.get().getAbsolutePosition();
        kVelocity = kAbsoluteEncoder.get().getVelocity();
        kTemperature = kMotorController.getDeviceTemp();
        kStatorCurrent = kMotorController.getStatorCurrent();
        kSupplyCurrent = kMotorController.getSupplyCurrent();
        kVoltage = kMotorController.getMotorVoltage();
    }

    /**
     * Updates all of the inputs of the motor if you want to use AK logging.
     * @param toUpdate The inputs to update.
     */
    public void updateInputs(motorInputs toUpdate) {
        toUpdate.isOk = BaseStatusSignal.refreshAll(
            kPosition,
            kVelocity,
            kTemperature,
            kStatorCurrent,
            kSupplyCurrent,
            kVoltage
        ).isOK();
        
        toUpdate.relativePositionRotations = kPosition.getValueAsDouble();
        toUpdate.velocityRotationsPerSecond = kVelocity.getValueAsDouble();
        toUpdate.temperatureCelcius = kTemperature.getValueAsDouble();
        toUpdate.statorCurrent = kStatorCurrent.getValueAsDouble();
        toUpdate.supplyCurrent = kSupplyCurrent.getValueAsDouble();
        toUpdate.supplyVoltage = kVoltage.getValueAsDouble();
    }

    /**
     * Sets the rotation goal of the motor.
     * @param rotations How many rotations the motor should target.
     */
    public void setMotorRotations(double rotations) {
        kMotorController.setControl(kPositionOut.withPosition(rotations).withSlot(0));
    }
    
    /**
     * Sets the RPS goal of the motor.
     * @param rps How many rotations per second the motor should target.
     */
    public void setMotorRotationsPerSecond(double rps) {
        kMotorController.setControl(kVelocityOut.withVelocity(rps).withSlot(0));
    }

    /**
     * Sets the voltage applied to the motor. This method does not take advantage of PID (Obviously)
     * @param voltage The amount of volts that will be applied to the motor
     */
    public void setMotorVoltage(double voltage) {
        kMotorController.setControl(kVoltageOut.withOutput(voltage));
    }

    /**
     * Stops the motor and sets it to its neutral state, Brake/Coast.
     */
    public void stop() {
        kMotorController.setControl(new NeutralOut());
    }

    /**
     * Resets the relative encoder inside of the robot (If you're using an absolute encoder you can access it through the CANCoder class making this method useless).
     */
    public void reset() {
        kMotorController.setPosition(0.0d);
    }

    /**
     * Reapplies the configuration, I'm adding this because I don't know if the motor controller creates a deep or shallow copy of the motor configuration, so you may need to reapply the config.
     */
    public void reapplyConfiguration() {
        kMotorController.getConfigurator().apply(kConfig);
    }
}
