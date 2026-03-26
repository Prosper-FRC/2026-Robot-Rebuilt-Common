package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.Shooter.ShooterConstants.HooderGains;
import edu.wpi.first.math.util.Units;

public class HooderIOSparkMax implements HooderIO {
    private final SparkMax kHooderMotor;
    private final CANcoder kHooderCancoder;
    private final CANcoderConfiguration kCancoderConfig = new CANcoderConfiguration();
    private StatusSignal<Angle> kHooderPosition;

    @AutoLogOutput(key = "Hooder/PositionRadians")
    private double kHooderPositionRadians = 0.0;
    
    @AutoLogOutput(key = "Hooder/PositionGoalRadians")
    private double kHooderPositionGoalRadians = 0.0;
    
    // Hooder feedforward and feedback declarations
    private final ProfiledPIDController hooderPIDController;
    private ArmFeedforward hooderFeedforward;

    public HooderIOSparkMax(HooderGains hooderGains) {
        ShooterConstants constants = ShooterConstants.getInstance();

        kHooderMotor = new SparkMax(constants.kHooderMotorID, MotorType.kBrushless);
        kHooderCancoder = new CANcoder(constants.kHooderCancoderID);

        kHooderCancoder.getConfigurator().apply(kCancoderConfig);
        kHooderPosition = kHooderCancoder.getPosition();

        SparkMaxConfig config = new SparkMaxConfig();

        config.softLimit.forwardSoftLimit(0.25);
        config.softLimit.forwardSoftLimitEnabled(true);

        config.softLimit.reverseSoftLimit(0.01);
        config.softLimit.reverseSoftLimitEnabled(true);

        kHooderMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Hooder feedforward and feedback initialization
        hooderPIDController = new ProfiledPIDController(
            hooderGains.p(), 
            hooderGains.i(),
            hooderGains.d(),
            new TrapezoidProfile.Constraints(hooderGains.maxVelocityRadiansPerSecond(), hooderGains.maxAccelerationRadiansPerSecondSquared())
        );

        hooderFeedforward = new ArmFeedforward(hooderGains.s(), hooderGains.g(), hooderGains.v(), hooderGains.a());
    }

    public double calculateHooderVolts(double goalPositionRadians) {
        double PIDVolts = hooderPIDController.calculate(kHooderPositionRadians, goalPositionRadians);
        double setpointVelocity = hooderPIDController.getSetpoint().velocity;
        double feedforwardVolts = hooderFeedforward.calculate(kHooderPositionRadians, setpointVelocity);

        return (PIDVolts + feedforwardVolts);
    }

    @Override
    public void setHooderVoltage(double volts) {
        kHooderMotor.setVoltage(volts);
    }
    
    @Override
    public void setHooderPositionRotationsGoal(Rotation2d newHoodPosition) {
        kHooderPositionGoalRadians = newHoodPosition.getRadians();
        hooderPIDController.setGoal(kHooderPositionGoalRadians);
    }

    @Override
    public double getHooderPositionRadiansGoal() {
        return kHooderPositionGoalRadians;
    }

    @Override
    public void stopHooder() {
        kHooderMotor.stopMotor();
    }

    @Override
    public void resetHooder() {
        kHooderCancoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(HooderInputs toUpdate) {
        kHooderPosition = kHooderCancoder.getAbsolutePosition();    
        kHooderPositionRadians = Units.rotationsToRadians(kHooderPosition.getValueAsDouble());

        kHooderMotor.setVoltage(calculateHooderVolts(kHooderPositionGoalRadians));

        toUpdate.hooderOk = BaseStatusSignal.refreshAll(kHooderPosition).isOK();
        toUpdate.hooderAngleRads = kHooderPosition.getValueAsDouble();
        toUpdate.hooderVelocityRPM = 0.0;
        toUpdate.hooderVoltage = kHooderMotor.getBusVoltage() * kHooderMotor.getAppliedOutput();
        toUpdate.hooderStatorCurrent = kHooderMotor.getOutputCurrent();
        toUpdate.hooderSupplyCurrent = kHooderMotor.getOutputCurrent();
    }
}