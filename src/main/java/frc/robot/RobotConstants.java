package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Subsystems.Vision.visionConstants.VisionConstants;
import frc.robot.Subsystems.Vision.visionConstants.VisionConstants5411;
import frc.robot.Subsystems.Vision.visionConstants.VisionConstants9105;
import frc.robot.Subsystems.Vision.visionConstants.VisionConstants9492;
import frc.robot.Subsystems.Vision.visionConstants.VisionConstantsSim;

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
    public final int kDriveControllerPort = 0;
    public final double kTimestep = 0.02d;

    private final VisionConstants kVisionConstants;

    // FALSE IT BEFORE COMPETITION
    public static final boolean kTuningMode = true;


    // Declare team specific constants here.

    private RobotConstants() {
        kTeamNumber = RobotController.getTeamNumber();
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
                kVisionConstants = new VisionConstants5411();
                break;
            case 9105:
                // Assign team specific constants.
                kVisionConstants = new VisionConstants9105();
                break;
            case 9492:
                // Assign team specific constants.
                kVisionConstants = new VisionConstants9492();
                break;
            case 0:
                // Assign sim constants
                kVisionConstants = new VisionConstantsSim();
                break;
            default:
                kVisionConstants = new VisionConstants();
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