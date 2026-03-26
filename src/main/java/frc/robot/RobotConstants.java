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
import frc.robot.Subsystems.Drive.Vision.CameraConstants;

public class RobotConstants {
    private static RobotConstants instance = null;

    public static enum mode {
        REAL,
        REPLAY,
        SIM
    };

    // Declare and Assign general constants here.
    public final int kTeamNumber;
    public final mode kMode;
    public final boolean kIsBlueAlliance;
    public final int kDriveControllerPort = 0;
    public final double kTimestep = 0.02d;
    
    public static final boolean kTuningMode = true;

    private final DriveConstants kDriveConstants;
    private final CameraConstants kCameraConstants;

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
                kCameraConstants = new CameraConstants();
                break;
            case 9105:
                kDriveConstants = new DriveConstants9105();
                kCameraConstants = new CameraConstants();
                break;
            case 9492:
                kDriveConstants = new DriveConstants9492();
                kCameraConstants = new CameraConstants();
                break;
            case 0:
                kDriveConstants = new DriveConstantsSim();
                kCameraConstants = new CameraConstants();
                break;
            default:
                kDriveConstants = new DriveConstants();
                kCameraConstants = new CameraConstants();
                break;
        }
    }

    public static DriveConstants DriveConstants() {
        return instance.kDriveConstants;
    }

    public static CameraConstants CameraConstants() {
        return instance.kCameraConstants;
    }

    public static RobotConstants Instance() {
        if (instance == null) {
            instance = new RobotConstants();
        }
        return instance;
    }
}