// package frc.robot.Subsystems.Shooter;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.hardware.CANcoder;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.units.measure.*;
// import frc.robot.Subsystems.Shooter.ShooterConstants;

// public class HooderIOSparkMax implements HooderIO {
//     private final SparkMax kHoodMotor;
//     private final CANcoder kHoodCancoder;
//     private final CANcoderConfiguration kCancoderConfig = new CANcoderConfiguration();
//     private final StatusSignal<Angle> kHoodPosition;

//     public HooderIOSparkMax() {
//         ShooterConstants constants = ShooterConstants.getInstance();

//         kHoodMotor = new SparkMax(constants.kHoodMotorId, MotorType.kBrushless);
//         kHoodCancoder = new CANcoder(constants.kHoodCancoderID);

//         kHoodCancoder.getConfigurator().apply(kCancoderConfig);
//         kHoodPosition = kHoodCancoder.getPosition();

//         SparkMaxConfig config = new SparkMaxConfig();

//         config.softLimit.forwardSoftLimit(0.25);
//         config.softLimit.forwardSoftLimitEnabled(true);

//         config.softLimit.reverseSoftLimit(0.01);
//         config.softLimit.reverseSoftLimitEnabled(true);

        
//         kHoodMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     @Override
//     public void updateInputs(HooderInputsAutoLogged toUpdate) {
       
//         toUpdate.isHoodOk = BaseStatusSignal.refreshAll(kHoodPosition).isOK();
//         toUpdate.hoodPositionRotations = kHoodPosition.getValueAsDouble();
//         toUpdate.hoodVelocityRPM = 0.0;
//         toUpdate.hoodVoltage = kHoodMotor.getBusVoltage() * kHoodMotor.getAppliedOutput();
//         toUpdate.hoodStatorCurrent = kHoodMotor.getOutputCurrent();
//         toUpdate.hoodSupplyCurrent = kHoodMotor.getOutputCurrent();
//     }

//     @Override
//     public void setVoltage(double volts) {
//         kHoodMotor.setVoltage(volts);
//     }

//     @Override
//     public void setHooderPositionRotationsGoal(Rotation2d goal) {
//         double error = goal.getRotations() - kHoodPosition.getValueAsDouble();
//         kHoodMotor.setVoltage(error * 5.0);
//     }

//     @Override
//     public void stopHooder() {
//         kHoodMotor.stopMotor();
//     }

//     @Override
//     public void resetHooder() {
//         kHoodCancoder.setPosition(0.0);
//     }
// }