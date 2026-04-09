package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants5411;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants9105;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants9492;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstantsSim;

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
    private final IntakeConstants kIntakeConstants;

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
                kIntakeConstants = new IntakeConstants5411();
                kDriveConstants = new DriveConstants();
                break;
            case 9105:
                kIntakeConstants = new IntakeConstants9105();
                kDriveConstants = new DriveConstants();
                break;
            case 9492:
                kIntakeConstants = new IntakeConstants9492();
                kDriveConstants = new DriveConstants();
                break;
            case 0:
                kIntakeConstants = new IntakeConstantsSim();
                kDriveConstants = new DriveConstants();
                break;
            default:
                kIntakeConstants = new IntakeConstants();
                kDriveConstants = new DriveConstants();
                break;
        }
    }

    public static DriveConstants DriveConstants() {
        return instance.kDriveConstants;
    }

    public static IntakeConstants IntakeConstants() {
        return instance.kIntakeConstants;
    }

    public static RobotConstants Instance() {
        if (instance == null) {
            instance = new RobotConstants();
        }
        return instance;
    }
}