package frc.robot.Subsystems.Drive.DriveConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class DriveConstants5411 extends DriveConstants {
    public DriveConstants5411() 
    {
        kFLModuleIDs = new moduleIDs(13, 23, 33);
        kFRModuleIDs = new moduleIDs(14, 24, 34);
        kBLModuleIDs = new moduleIDs(11, 21, 31);
        kBRModuleIDs = new moduleIDs(12, 22, 32);

        kModuleHardLimits = new moduleHardLimits(0.0508d, 6.12d/1.0d, 150.0d/7.0d, Units.inchesToMeters(26.5d));
        kModuleSoftLimits = new moduleSoftlimits(new moduleControllerLimits(0.05d, 2, 6.0d), 4.5d, 4.0d, 1.0d, 24.0, true, true);

        kFLModuleOffsets = new moduleOffsets(new Translation2d(kModuleHardLimits.trackDistanceMeters()/2, kModuleHardLimits.trackDistanceMeters()/2), Rotation2d.fromRotations(-0.471924), true);
        kFRModuleOffsets = new moduleOffsets(new Translation2d(kModuleHardLimits.trackDistanceMeters()/2, -kModuleHardLimits.trackDistanceMeters()/2), Rotation2d.fromRotations(0.355957), false);
        kBLModuleOffsets = new moduleOffsets(new Translation2d(-kModuleHardLimits.trackDistanceMeters()/2, kModuleHardLimits.trackDistanceMeters()/2), Rotation2d.fromRotations(-0.245605), true);
        kBRModuleOffsets = new moduleOffsets(new Translation2d(-kModuleHardLimits.trackDistanceMeters()/2, -kModuleHardLimits.trackDistanceMeters()/2), Rotation2d.fromRotations(0.240967), false);
    
        kXTranslationalConstraints = new TrapezoidProfile.Constraints(kModuleSoftLimits.absoluteMaxDriveVelocityMPS(), kModuleSoftLimits.maxLinearAccelerationMPS2());
        kXTranslationalController = new ProfiledPIDController(0.0d, 0.0d, 0.0d, kXTranslationalConstraints);
        kYTranslationalConstraints = new TrapezoidProfile.Constraints(kModuleSoftLimits.absoluteMaxDriveVelocityMPS(), kModuleSoftLimits.maxLinearAccelerationMPS2());
        kYTranslationalController = new ProfiledPIDController(0.0d, 0.0d, 0.0d, kYTranslationalConstraints);
        kHeadingConstraints = new TrapezoidProfile.Constraints(kModuleSoftLimits.absoluteMaxDriveVelocityMPS(), kModuleSoftLimits.maxLinearAccelerationMPS2());
        kHeadingController = new ProfiledPIDController(0.0d, 0.0d, 0.0d, kHeadingConstraints);
    }
}
