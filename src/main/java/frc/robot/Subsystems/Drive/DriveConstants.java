package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConstants;

public class DriveConstants {
    private static DriveConstants instance = null;
    public static record hardwareSpecifications(double kWheelRadiusMeters) {}
    public static record moduleOffsets(Translation2d FLPoseOffset, double FLRotationOffset, Translation2d FRPoseOffset, double FRRotationalOffset, Translation2d BLPoseOffset, double BLRotationalOffset, Translation2d BRPoseOffset, double BRRotationalOffset) {}
    public static record moduleIDs(int FLDriveID, int FLAzimuthID, int FRDriveID, int FRAzimtuhID, int BLDriveID, int BLAzimuthID, int BRDriveID, int BRAzimuthID) {}
    public static record softLimits(double maximumAngularVelocityRotations, double maximumLinearVelocityMPS) {}

    // Declare and Assign general constants here.
    public final double kDriveGearing = 1.0d;
    public final double kAzimuthGearing = 1.0d;
    public final double kMaxVoltage = 12.0d;

    // Declare team specific constants here.
    public final hardwareSpecifications kHardwareSpecifictions;
    public final moduleOffsets kModuleOffsets;
    public final moduleIDs kModuleIDs;
    public final softLimits kSoftLimits;
    public final PIDController kDrivePIDController;
    public final PIDController kAzimuthPIDController;

    private DriveConstants() {
        kModuleIDs = new moduleIDs(0, 0, 0, 0, 0, 0, 0, 0);
        switch (RobotConstants.getInstance().kTeamNumber) {
            case 5411:
                // Assign team specific constants.
                kHardwareSpecifictions = new hardwareSpecifications(0.0d);
                kModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d, new Translation2d(0.5d, 0.5d), 0.0d, new Translation2d(-0.5d, -0.5d), 0.0d, new Translation2d(0.5d, -0.5d), 0.0d);
                kDrivePIDController = new PIDController(0.05d, 0.0d, 0.025);
                kAzimuthPIDController = new PIDController(0.05d, 0.0d, 0.025d);
                kSoftLimits = new softLimits(0.5d, 6.0d);
                break;
            case 9105:
                // Assign team specific constants.
                kHardwareSpecifictions = new hardwareSpecifications(0.0d);
                kModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d, new Translation2d(0.5d, 0.5d), 0.0d, new Translation2d(-0.5d, -0.5d), 0.0d, new Translation2d(0.5d, -0.5d), 0.0d);
                kDrivePIDController = new PIDController(0.05d, 0.0d, 0.025);
                kAzimuthPIDController = new PIDController(0.05d, 0.0d, 0.025d);
                kSoftLimits = new softLimits(0.5d, 6.0d);
                break;
            case 9492:
                // Assign team specific constants.
                kHardwareSpecifictions = new hardwareSpecifications(0.0d);
                kModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d, new Translation2d(0.5d, 0.5d), 0.0d, new Translation2d(-0.5d, -0.5d), 0.0d, new Translation2d(0.5d, -0.5d), 0.0d);
                kDrivePIDController = new PIDController(0.05d, 0.0d, 0.025);
                kAzimuthPIDController = new PIDController(0.05d, 0.0d, 0.025d);
                kSoftLimits = new softLimits(0.5d, 6.0d);
                break;
            case 0:
                // Assign sim constants
                kHardwareSpecifictions = new hardwareSpecifications(0.5d);
                kModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d, new Translation2d(0.5d, 0.5d), 0.0d, new Translation2d(-0.5d, -0.5d), 0.0d, new Translation2d(0.5d, -0.5d), 0.0d);
                kDrivePIDController = new PIDController(0.05d, 0.0d, 0.025);
                kAzimuthPIDController = new PIDController(0.05d, 0.0d, 0.025d);
                kSoftLimits = new softLimits(0.5d, 6.0d);
                break;
            default:
                kHardwareSpecifictions = new hardwareSpecifications(0.0d);
                kModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d, new Translation2d(0.5d, 0.5d), 0.0d, new Translation2d(-0.5d, -0.5d), 0.0d, new Translation2d(0.5d, -0.5d), 0.0d);
                kDrivePIDController = new PIDController(0.05d, 0.0d, 0.025);
                kAzimuthPIDController = new PIDController(0.05d, 0.0d, 0.025d);
                kSoftLimits = new softLimits(0.5d, 6.0d);
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
