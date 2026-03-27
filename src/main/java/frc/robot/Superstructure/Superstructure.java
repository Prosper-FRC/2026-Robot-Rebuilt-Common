package frc.robot.Superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroPigeon2;
import frc.robot.Subsystems.Drive.GyroSim;
import frc.robot.Subsystems.Drive.ModuleSim;
import frc.robot.Subsystems.Drive.ModuleTalonFX;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerTalonFX;
import frc.robot.Subsystems.Indexer.IndexerConstantsMain.IndexerConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeSim;
import frc.robot.Subsystems.Intake.IntakeTalonFX;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants;

public class Superstructure {
    private final int kTeamNumber = RobotConstants.Instance().kTeamNumber;

    public final Drive kDrive;
    public final Intake kIntake;
    public final Indexer kIndexer;

    private final DriveConstants kDConsts = RobotConstants.DriveConstants();
    private final IntakeConstants kIntConsts = RobotConstants.IntakeConstants();
    private final IndexerConstants kIndConsts = RobotConstants.IndexerConstants();

    public Superstructure(boolean useDrive, boolean useIntake, boolean useIndexer, boolean useShooter) {
        if(kTeamNumber == 0) {
            // Sim init
            kDrive = new Drive(new ModuleSim(),
            new ModuleSim(), 
            new ModuleSim(), 
            new ModuleSim(), 
            new GyroSim());
            kIntake = new Intake(new IntakeSim());
            kIndexer = new Indexer(new IndexerTalonFX());
        } else {
            if(useDrive) {
                kDrive = new Drive(
                    new ModuleTalonFX(kDConsts.kFLModuleIDs, kDConsts.kFLModuleOffsets, kDConsts.kModuleGains, kDConsts.kCANBusInstance), 
                    new ModuleTalonFX(kDConsts.kFRModuleIDs, kDConsts.kFRModuleOffsets, kDConsts.kModuleGains, kDConsts.kCANBusInstance), 
                    new ModuleTalonFX(kDConsts.kBLModuleIDs, kDConsts.kBLModuleOffsets, kDConsts.kModuleGains, kDConsts.kCANBusInstance), 
                    new ModuleTalonFX(kDConsts.kBRModuleIDs, kDConsts.kBRModuleOffsets, kDConsts.kModuleGains, kDConsts.kCANBusInstance), 
                    new GyroPigeon2(kDConsts.kGyroID, kDConsts.kGyroOffsets, kDConsts.kCANBusInstance));
            } else {
                kDrive = Drive.NoOp;
            }
            if(useIntake) {
                kIntake = new Intake(new IntakeTalonFX(kIntConsts.kIntakeIDs));
            } else {
                kIntake = Intake.NoOp;
            }
            if(useIndexer) {
                kIndexer = new Indexer(new IndexerTalonFX());
            } else {
                kIndexer = Indexer.NoOp;
            }
        }
    }
}
