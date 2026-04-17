package frc.robot.Subsystems.Drive.DriveConstants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final record SwerveModuleIds(int driveId, int azimuthId, int cancoderId) {}
    public static final record SwerveModuleGains(double kP, double kI, double kD, double kV, double kS, double kA) {}
    public static final record SwerveModuleMotionMagicGains(double drive_cruise_velocity, double drive_acceleration) {}
    public static final record SwerveModuleCurrentLimits(double driveStatorLimit, double driveSupplyLimit, double azimuthStatorLimit, double azimuthSupplyLimit) {}
    public static final record SwerveModuleOutputConfigs(boolean isCCWPositive, boolean isBrakedNeutral) {}
    public static final record SwerveModuleHardware(double driveSideLengthsMeters,  double wheelRadiusMeters, double driveGearReduction, double azimuthGearReduction) {}
    public static final record DriveSoftLimits(DriveControllerLimits controllerLimits, double maxLinearVelocityMPS, double maxAngularVelocityRPS ) {}
    public static final record DriveControllerLimits(double deadband, double inputExponent, double inputRateLimiter) {}

    public final SwerveModuleIds kFLModuleIds = new SwerveModuleIds(11, 21, 31);
    public final SwerveModuleIds kFRModuleIds = new SwerveModuleIds(12, 22, 32);
    public final SwerveModuleIds kBLModuleIds = new SwerveModuleIds(13, 23, 33);
    public final SwerveModuleIds kBRModuleIds = new SwerveModuleIds(14, 24, 34);
    public final int kGyroId = 40;

    public final double kFLModuleOffset = -0.244d;
    public final double kFRModuleOffset = 0.382d;
    public final double kBLModuleOffset = -0.223d;
    public final double kBRModuleOffset = -0.495d;
    public final Rotation3d kGyroOffsets = new Rotation3d();

    public final SwerveModuleGains kDriveGains = new SwerveModuleGains(
        0.75d, 0.0d, 0.0d, 
        0.7d, 0.0d, 0.0d
    );
    public final SwerveModuleGains kAzimuthGains = new SwerveModuleGains(
        22.5d, 0.0d, 0.0d, 
        0.0d, 0.0d, 0.0d
    );

    public final PIDController kHeadingController = new PIDController(1.5d, 0.0d, 0.0d);

    public final PIDController kXTranslationalController = new PIDController(1.5d, 0.0d, 0.0d);
    public final PIDController kYTranslationalController = new PIDController(1.5d, 0.0d, 0.0d);
    
    public final SwerveModuleCurrentLimits kModuleCurrentLimits = new SwerveModuleCurrentLimits(
        80.0d, 60.0d, 
        45.0d, 35.0d
    );

    // public final SwerveModuleMotionMagicGains kMotionMagicGains = new SwerveModuleMotionMagicGains(
    //     4.5d,
    //     28.0d
    // );

    public final SwerveModuleOutputConfigs kDriveOutputConfigs = new SwerveModuleOutputConfigs(
        false, 
        true
    );
    public final SwerveModuleOutputConfigs kAzimuthOutputConfigs = new SwerveModuleOutputConfigs(
        false, 
        true
    );

    public final SwerveModuleHardware kModuleHardware = new SwerveModuleHardware(
        Units.inchesToMeters(26.5d),
        Units.inchesToMeters(2.0d),
        6.12d/1.0d, 
        150.0d/7.0d
    );

    public final CANBus kCanbus = new CANBus("drivebase");

    public final DriveSoftLimits kDriveSoftLimits = new DriveSoftLimits(
        new DriveControllerLimits(0.1d, 2.0d, 6.0d),
        4.5d,
        1.0d
    );

    public final PIDController kDriveControllerSim = new PIDController(0.55d, 0.0d, 0.0d);
    public final SimpleMotorFeedforward kDriveFeedForwardSim = new SimpleMotorFeedforward(0.0d, 0.65d);

    public final PIDController kAzimuthControllerSim = new PIDController(37.5d, 0.0d, 0.5d);
}
