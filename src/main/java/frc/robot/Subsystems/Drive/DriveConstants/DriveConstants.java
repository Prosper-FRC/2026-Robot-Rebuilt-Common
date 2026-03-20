package frc.robot.Subsystems.Drive.DriveConstants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public moduleIDs kFLModuleIDs;
    public moduleIDs kFRModuleIDs;
    public moduleIDs kBLModuleIDs;
    public moduleIDs kBRModuleIDs;
    public int kGyroID = 10;

    public moduleHardLimits kModuleHardLimits;
    public moduleSoftlimits kModuleSoftLimits;

    public double sniperModeScalar = 0.2d;
    
    public moduleOffsets kFLModuleOffsets;
    public moduleOffsets kFRModuleOffsets;    
    public moduleOffsets kBLModuleOffsets;
    public moduleOffsets kBRModuleOffsets;

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

    public TrapezoidProfile.Constraints kXTranslationalConstraints;
    public ProfiledPIDController kXTranslationalController;
    public TrapezoidProfile.Constraints kYTranslationalConstraints;
    public ProfiledPIDController kYTranslationalController;
    public TrapezoidProfile.Constraints kHeadingConstraints;
    public ProfiledPIDController kHeadingController;

    public DriveConstants() {}
}
