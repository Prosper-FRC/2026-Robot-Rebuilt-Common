package frc.robot.Subsystems.Drive.DriveConstants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public final record moduleIDs(int driveID, int azimuthID, int CANcoderID) {}
    public final record moduleOffsets(Translation2d translationalOffset, Rotation2d rotationalOffset, boolean isInverted) {}
    public final record moduleHardLimits(double wheelRadiusMeters, double driveGearRatio, double azimuthGearRatio, double trackDistanceMeters) {}
    public final record moduleControllerLimits(double controllerDeadband, int controllerInputExponent, double controllerInputRateLimiter) {}
    public final record moduleSoftlimits(moduleControllerLimits controllerLimits, double maxLinearVelocityMPS, double maxLinearAccelerationMPS2, double maxAngularVelocityRPS, double absoluteMaxDriveVelocityMPS, boolean isDriveBraked, boolean isAzimuthBraked) {}
    public final record moduleCurrentLimits(double driveStatorCurrentLimit, double driveSupplyCurrentLimit, double azimuthStatorCurrentLimit, double azimuthSupplyCurrentLimit) {}
    public final record moduleVoltageLimits(double driveVoltagePeakRange, double azimuthVoltagePeakRange) {}
    public final record motorGains(double kP, double kI, double kD, double kS, double kV, double kA) {}
    public final record motionMagicGains(double maxCruiseVelocity, double maxAcceleration) {}
    public final record moduleGains(motorGains driveGains, motorGains azimuthGains, motionMagicGains driveMMGains) {}

    public final record gyroOffsets(double roll, double pitch, double yaw) {}

    public moduleIDs kFLModuleIDs = new moduleIDs(11, 21, 31);
    public moduleIDs kFRModuleIDs = new moduleIDs(12, 22, 32);
    public moduleIDs kBLModuleIDs = new moduleIDs(13, 23, 33);
    public moduleIDs kBRModuleIDs = new moduleIDs(14, 24, 34);
    public int kGyroID = 10;

    public moduleHardLimits kModuleHardLimits = new moduleHardLimits(0.0508d, 6.12d/1.0d, 150.0d/7.0d, Units.inchesToMeters(26.5d));
    public moduleSoftlimits kModuleSoftLimits = new moduleSoftlimits(new moduleControllerLimits(0.05d, 2, 6.0d), 4.5d, 4.0d, 1.0d, 24.0, true, true);

    public double sniperModeScalar = 0.2d;

    public moduleOffsets kFLModuleOffsets = new moduleOffsets(new Translation2d(-kModuleHardLimits.trackDistanceMeters/2, kModuleHardLimits.trackDistanceMeters/2), Rotation2d.fromRotations(0.43359375), true);
    public moduleOffsets kFRModuleOffsets = new moduleOffsets(new Translation2d(kModuleHardLimits.trackDistanceMeters/2, kModuleHardLimits.trackDistanceMeters/2), Rotation2d.fromRotations(-0.42333984375), false);
    public moduleOffsets kBLModuleOffsets = new moduleOffsets(new Translation2d(-kModuleHardLimits.trackDistanceMeters/2, -kModuleHardLimits.trackDistanceMeters/2), Rotation2d.fromRotations(0.255615234375), true);
    public moduleOffsets kBRModuleOffsets = new moduleOffsets(new Translation2d(kModuleHardLimits.trackDistanceMeters/2, -kModuleHardLimits.trackDistanceMeters/2), Rotation2d.fromRotations(-0.412841796875), false);

    public gyroOffsets kGyroOffsets = new gyroOffsets(0.0d, 0.0d, 0.0d);

    public CANBus kCANBusInstance = new CANBus("drivebase");

    // Recommended as default values for swerve by CTRE.
    public moduleGains kModuleGains = new moduleGains(
        new motorGains(0.75d, 0, 0, 0, 0.6, 0),
        new motorGains(30.0d, 0, 0.5d, 0.1d, 3.1d, 0.0d),
        new motionMagicGains(14.1d, 9.0d)
    );

    public moduleCurrentLimits kModuleCurrentLimits = new moduleCurrentLimits(60, 80, 30, 45);
    public moduleVoltageLimits kModuleVoltageLimits = new moduleVoltageLimits(12.0d, 12.0d);

    public PIDController kSimDrivePID = new PIDController(0.5d, 0.0d, 0.0d);
    public SimpleMotorFeedforward kSimDriveFeedforward = new SimpleMotorFeedforward(0.0d, 0.75d);
    public PIDController kSimAzimuthPID = new PIDController(37.5d, 0.0d, 0.5d);

    public DriveConstants() {}
}
