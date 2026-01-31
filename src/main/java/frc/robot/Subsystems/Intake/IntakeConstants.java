package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.RobotController;

public class IntakeConstants {
    private static IntakeConstants instance = null;

    // Declare and Assign general constants here.
    public final int kTeamNumber;

    // Declare team specific constants here.

    private IntakeConstants() {
        kTeamNumber = RobotController.getTeamNumber();

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
            case 0:
                // Assign sim constants
                break;
            default:
                break;
        }
    }

    public static IntakeConstants getInstance() {
        // Using a null check so that the instance is created at the proper time.
        if (instance == null) {
            instance = new IntakeConstants();
        }
        return instance;
    }
}