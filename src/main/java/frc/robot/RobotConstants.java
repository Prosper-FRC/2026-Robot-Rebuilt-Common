package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public class RobotConstants {
    private static RobotConstants instance = null;

    public static enum mode {
        REAL,
        REPLAY,
        SIM
    };

    // Declare and Assign general constants here.
    public final int kTeamNumber;
    public static final mode kCurrentMode = RobotBase.isReal() ? mode.REAL : mode.SIM;
    public final int kDriveControllerPort = 0;
    public final double kTimestep = 0.02d;

    // Declare team specific constants here.

    private RobotConstants() {
        kTeamNumber = RobotController.getTeamNumber();

        switch (kTeamNumber) {
            case 5411:
                break;
            case 9105:
                break;
            case 9492:
                break;
            case 0:
                break;
            default:
                break;
        }
    }
    public static RobotConstants Instance() {
        // Using a null check so that the instance is created at the proper time.
        if (instance == null) {
            instance = new RobotConstants();
        }
        return instance;
    }

    public static RobotConstants getInstance() {
        // Using a null check so that the instance is created at the proper time.
        if (instance == null) {
            instance = new RobotConstants();
        }
        return instance;
    }
}