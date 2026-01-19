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
    public final int kTeamNumberOverride = 5411;
    public final mode kMode;

    // Declare team specific constants here.

    private RobotConstants() {
        // When in sim I don't think the team number can be retrieved since its based off of the RIO team number, not of the json pereferences.
        kTeamNumber = RobotController.getTeamNumber() != 0 ? RobotController.getTeamNumber() : kTeamNumberOverride;
        if(RobotBase.isReal()) {
            kMode = mode.REAL;
        } else if(RobotBase.isSimulation()) {
            kMode = mode.SIM;
        } else {
            kMode = mode.REPLAY;
        }

        switch (kTeamNumber) {
            case 5411:
                // Assign team specific constants.
                break;
            case 9105:
                // Assign team specific constants.
                break;
            case 9492:
                // Assign team specific constants.
                break;
            default:
                break;
        }
    }

    public static RobotConstants getInstance() {
        // Using a null check so that the instance is created at the proper time.
        if (instance == null) {
            instance = new RobotConstants();
        }
        return instance;
    }
}
