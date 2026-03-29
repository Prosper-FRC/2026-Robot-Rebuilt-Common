package frc.robot.Factories;

import frc.robot.RobotConstants;
import frc.robot.Factories.SubsystemFactory.SubsystemType;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroPigeon2;
import frc.robot.Subsystems.Drive.GyroSim;
import frc.robot.Subsystems.Drive.ModuleSim;
import frc.robot.Subsystems.Drive.ModuleTalonFX;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants;

public class DriveFactory {
    public static final DriveConstants kDriveConstants = RobotConstants.DriveConstants();

    public static Drive create(SubsystemType type) {
        switch(type) {
            case REAL:
                return new Drive(
                    new ModuleTalonFX(kDriveConstants.kFLModuleIDs, kDriveConstants.kFLModuleOffsets, kDriveConstants.kModuleGains, kDriveConstants.kCANBusInstance),
                    new ModuleTalonFX(kDriveConstants.kFRModuleIDs, kDriveConstants.kFRModuleOffsets, kDriveConstants.kModuleGains, kDriveConstants.kCANBusInstance),
                    new ModuleTalonFX(kDriveConstants.kBLModuleIDs, kDriveConstants.kBLModuleOffsets, kDriveConstants.kModuleGains, kDriveConstants.kCANBusInstance),
                    new ModuleTalonFX(kDriveConstants.kBRModuleIDs, kDriveConstants.kBRModuleOffsets, kDriveConstants.kModuleGains, kDriveConstants.kCANBusInstance),
                    new GyroPigeon2(kDriveConstants.kGyroID, kDriveConstants.kGyroOffsets, kDriveConstants.kCANBusInstance)
                );
            case SIM:
                return new Drive(
                    new ModuleSim(),
                    new ModuleSim(),
                    new ModuleSim(),
                    new ModuleSim(),
                    new GyroSim()
                );
            default:
                return Drive.NoOp;
        }
    }
}
