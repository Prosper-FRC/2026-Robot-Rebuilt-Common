package frc.robot.Subsystems.Drive.DriveConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstantsSim extends DriveConstants {
    public DriveConstantsSim() 
    {
        kFLModuleIDs = new moduleIDs(11, 21, 31);
        kFRModuleIDs = new moduleIDs(12, 22, 32);
        kBLModuleIDs = new moduleIDs(13, 23, 33);
        kBRModuleIDs = new moduleIDs(14, 24, 34);

        kModuleHardLimits = new moduleHardLimits(0.0508d, 6.12d/1.0d, 150.0d/7.0d, Units.inchesToMeters(26.5d));
        kModuleSoftLimits = new moduleSoftlimits(new moduleControllerLimits(0.05d, 2, 6.0d), 4.5d, 4.0d, 1.0d, 24.0, true, true);

        kFLModuleOffsets = new moduleOffsets(new Translation2d(kModuleHardLimits.trackDistanceMeters()/2, kModuleHardLimits.trackDistanceMeters()/2), Rotation2d.fromRotations(0.43359375), true);
        kFRModuleOffsets = new moduleOffsets(new Translation2d(kModuleHardLimits.trackDistanceMeters()/2, -kModuleHardLimits.trackDistanceMeters()/2), Rotation2d.fromRotations(-0.42333984375), false);
        kBLModuleOffsets = new moduleOffsets(new Translation2d(-kModuleHardLimits.trackDistanceMeters()/2, kModuleHardLimits.trackDistanceMeters()/2), Rotation2d.fromRotations(0.255615234375), true);
        kBRModuleOffsets = new moduleOffsets(new Translation2d(-kModuleHardLimits.trackDistanceMeters()/2, -kModuleHardLimits.trackDistanceMeters()/2), Rotation2d.fromRotations(-0.412841796875), false);
    }
}
