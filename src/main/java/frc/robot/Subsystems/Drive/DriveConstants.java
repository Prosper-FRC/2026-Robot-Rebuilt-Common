package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConstants;

public class DriveConstants {
    private static DriveConstants instance = null;
    public static record hardwareSpecifications(double kWheelRadiusMeters) {}
    public static record moduleOffsets(Translation2d FLPoseOffset, double FLRotationOffset, Translation2d FRPoseOffset, double FRRotationalOffset, Translation2d BLPoseOffset, double BLRotationalOffset, Translation2d BRPoseOffset, double BRRotationalOffset) {}

    // Declare and Assign general constants here.
    public final double kGearing = 1.0d;
    public final double kMaxVoltage = 12.0d;

    // Declare team specific constants here.
    public final hardwareSpecifications kHardwareSpecifictions;
    public final moduleOffsets kModuleOffsets;

    private DriveConstants() {
        switch (RobotConstants.getInstance().kTeamNumber) {
            case 5411:
                // Assign team specific constants.
                kHardwareSpecifictions = new hardwareSpecifications(0.0d);
                kModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d, new Translation2d(0.5d, 0.5d), 0.0d, new Translation2d(-0.5d, -0.5d), 0.0d, new Translation2d(0.5d, -0.5d), 0.0d);
                break;
            case 9105:
                // Assign team specific constants.
                kHardwareSpecifictions = new hardwareSpecifications(0.0d);
                kModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d, new Translation2d(0.5d, 0.5d), 0.0d, new Translation2d(-0.5d, -0.5d), 0.0d, new Translation2d(0.5d, -0.5d), 0.0d);
                break;
            case 9492:
                // Assign team specific constants.
                kHardwareSpecifictions = new hardwareSpecifications(0.0d);
                kModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d, new Translation2d(0.5d, 0.5d), 0.0d, new Translation2d(-0.5d, -0.5d), 0.0d, new Translation2d(0.5d, -0.5d), 0.0d);
                break;
            case 0:
                // Assign sim constants
                kHardwareSpecifictions = new hardwareSpecifications(0.5d);
                kModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d, new Translation2d(0.5d, 0.5d), 0.0d, new Translation2d(-0.5d, -0.5d), 0.0d, new Translation2d(0.5d, -0.5d), 0.0d);
                break;
            default:
                kHardwareSpecifictions = new hardwareSpecifications(0.0d);
                kModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d, new Translation2d(0.5d, 0.5d), 0.0d, new Translation2d(-0.5d, -0.5d), 0.0d, new Translation2d(0.5d, -0.5d), 0.0d);
                break;
        }
    }

    public static DriveConstants getInstance() {
        // Using a null check so that the instance is created at the proper time.
        if (instance == null) {
            instance = new DriveConstants();
        }
        return instance;
    }
}
