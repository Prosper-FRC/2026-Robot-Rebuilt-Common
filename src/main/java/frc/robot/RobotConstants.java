package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants;
import frc.robot.Subsystems.Indexer.IndexerConstants.IndexerConstants;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants;
import frc.robot.Subsystems.Shooter.ShooterConstants.ShooterConstants;

public class RobotConstants {
    private static RobotConstants instance = null;

    public static enum mode {
        REAL,
        REPLAY,
        SIM
    };

    public SendableChooser<Boolean> kChooser = new SendableChooser<Boolean>();

    public final int kTeamNumber;
    public final mode kMode;
    public final boolean kIsBlueAlliance;
    public final int kDriveControllerPort = 0;
    public final int kOperatorControllerPort = 1;
    public final double kTimestep = 0.02d;
    
    private final DriveConstants kDriveConstants;
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

        kDriveConstants = new DriveConstants();
        kIndexerConstants = new IndexerConstants();
        kIntakeConstants = new IntakeConstants();
        kShooterConstants = new ShooterConstants();
    }

    public static DriveConstants DriveConstants() {
        return instance.kDriveConstants;
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