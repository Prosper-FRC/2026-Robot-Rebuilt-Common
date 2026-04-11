package frc.robot.Factories;

import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants;
import frc.robot.Subsystems.Drive.Gyro.GyroPigeon;
import frc.robot.Subsystems.Drive.Gyro.GyroSim;
import frc.robot.Subsystems.Drive.SwerveModule.SwerveModuleSim;
import frc.robot.Subsystems.Drive.SwerveModule.SwerveModuleTalonFX;

public class DriveFactory {
    private static final DriveConstants kConstants = RobotConstants.DriveConstants();

    private DriveFactory() {}

    public static Drive createReal() {
        return new Drive(
            new SwerveModuleTalonFX(
                kConstants.kFLModuleIds,
                kConstants.kFLModuleOffset,
                kConstants.kCanbus
            ), 
            new SwerveModuleTalonFX(
                kConstants.kFRModuleIds,
                kConstants.kFRModuleOffset,
                kConstants.kCanbus
            ), 
            new SwerveModuleTalonFX(
                kConstants.kBLModuleIds,
                kConstants.kBLModuleOffset,
                kConstants.kCanbus
            ), 
            new SwerveModuleTalonFX(
                kConstants.kBRModuleIds,
                kConstants.kBRModuleOffset,
                kConstants.kCanbus
            ), 
            new GyroPigeon(kConstants.kGyroId, kConstants.kGyroOffsets, kConstants.kCanbus)
        );
    }
    
    public static Drive createSim() {
        return new Drive(
            new SwerveModuleSim(), 
            new SwerveModuleSim(), 
            new SwerveModuleSim(), 
            new SwerveModuleSim(), 
            new GyroSim()
        );
    }

    public static Drive createNoOp() {
        return Drive.NoOp;
    }
}