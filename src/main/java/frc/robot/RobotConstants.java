package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants5411;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants9105;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants9492;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstantsSim;
import frc.robot.Subsystems.Drive.Vision.visionConstants.VisionConstants;
import frc.robot.Subsystems.Drive.Vision.visionConstants.VisionConstants5411;
import frc.robot.Subsystems.Drive.Vision.visionConstants.VisionConstants9105;
import frc.robot.Subsystems.Drive.Vision.visionConstants.VisionConstants9492;
import frc.robot.Subsystems.Indexer.IndexerConstantsMain.IndexerConstants;
import frc.robot.Subsystems.Indexer.IndexerConstantsMain.IndexerConstants5411;
import frc.robot.Subsystems.Indexer.IndexerConstantsMain.IndexerConstants9105;
import frc.robot.Subsystems.Indexer.IndexerConstantsMain.IndexerConstants9492;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants5411;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants9105;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants9492;
import frc.robot.Subsystems.Shooter.ShooterConstants.ShooterConstants;
import frc.robot.Subsystems.Shooter.ShooterConstants.ShooterConstants5411;
import frc.robot.Subsystems.Shooter.ShooterConstants.ShooterConstants9105;
import frc.robot.Subsystems.Shooter.ShooterConstants.ShooterConstants9492;

public class RobotConstants {
    private static RobotConstants instance = null;

    public static enum mode {
        REAL,
        REPLAY,
        SIM
    };

    public final int kTeamNumber;
    public final mode kMode;
    public final boolean kIsBlueAlliance;
    public final int kDriveControllerPort = 0;
    public final double kTimestep = 0.02d;

    private final DriveConstants kDriveConstants;
    private final VisionConstants kVisionConstants;
    private final IndexerConstants kIndexerConstants;
    private final IntakeConstants kIntakeConstants;
    private final ShooterConstants kShooterConstants;

    private RobotConstants() {
        kTeamNumber = RobotController.getTeamNumber();
        if(RobotBase.isReal()) {
            kMode = mode.REAL;
        } else if(RobotBase.isSimulation()) {
            kMode = mode.SIM;
        } else {
            kMode = mode.REPLAY;
        }
        kIsBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;

        switch (kTeamNumber) {
            case 5411:
                kDriveConstants = new DriveConstants5411();
                kVisionConstants = new VisionConstants5411();
                kIndexerConstants = new IndexerConstants5411();
                kIntakeConstants = new IntakeConstants5411();
                kShooterConstants = new ShooterConstants5411();
                break;
            case 9105:
                kDriveConstants = new DriveConstants9105();
                kVisionConstants = new VisionConstants9105();
                kIndexerConstants = new IndexerConstants9105();
                kIntakeConstants = new IntakeConstants9105();
                kShooterConstants = new ShooterConstants9105();
                break;
            case 9492:
                kDriveConstants = new DriveConstants9492();
                kVisionConstants = new VisionConstants9492();
                kIndexerConstants = new IndexerConstants9492();
                kIntakeConstants = new IntakeConstants9492();
                kShooterConstants = new ShooterConstants9492();
                break;
            case 0:
                kDriveConstants = new DriveConstantsSim();
                kVisionConstants = new VisionConstants();
                kIndexerConstants = new IndexerConstants();
                kIntakeConstants = new IntakeConstants();
                kShooterConstants = new ShooterConstants();
                break;
            default:
                kDriveConstants = new DriveConstants();
                kVisionConstants = new VisionConstants();
                kIndexerConstants = new IndexerConstants();
                kIntakeConstants = new IntakeConstants();
                kShooterConstants = new ShooterConstants();
                break;
        }
    }

    public static DriveConstants DriveConstants() {
        return instance.kDriveConstants;
    }

    public static VisionConstants VisionConstants() {
        return instance.kVisionConstants;
    }

    public static IndexerConstants IndexerConstants() {
        return instance.kIndexerConstants;
    }

    public static IntakeConstants IntakeConstants() {
        return instance.kIntakeConstants;
    } 

    public static ShooterConstants ShooterConstants() {
        return instance.kShooterConstants;
    }

    public static RobotConstants Instance() {
        if (instance == null) {
            instance = new RobotConstants();
        }
        return instance;
    }
}