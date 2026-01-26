package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConstants;

public class DriveConstants {
    private static DriveConstants instance = null;

    // Defining records.
    public static record moduleOffsets(Translation2d poseOffset, double rotationOffset) {}
    public static record moduleIDs(int driveID, int azimuthID, int canCoderID) {}
    public static record softLimits(double maximumAngularVelocityRotations, double maximumLinearVelocityMPS) {}
    public static record currentConfig(double statorCurrentLimit, boolean useStatorCurrentLimit, double supplyCurrentLimit, boolean useSupplyCurrentLimit, double maxForwardTorque, double maxReverseTorque, double neutralDeadbandTorque) {}
    public static record motorOutputConfig(boolean isInverted, boolean isBraked, double peakForwardDutyCycle, double peakReverseDutyCycle) {}
    public static record voltageConfig(double peakForwardVoltage, double peakReverseVoltage) {}
    public static record feedforwardGains(double kA, double kS, double kV) {}

    // Declare and Assign general constants here.
    public final double kDriveGearing = 6.5/1d;
    public final double kAzimuthGearing = 21.43/1;
    public final double kRobotMassKG = 68.0d;
    public final double kMaxVoltage = 12.0d;
    public final double kWheelRadiusMeters = 0.05d;
    public final int kGyroID = 0;
    public final SimpleMotorFeedforward kSimDriveFeedforward = new SimpleMotorFeedforward(0.0d, 0.665d);
    // TODO Make these constants actually do something.
    public final currentConfig kDriveCurrentConfig = null;
    public final motorOutputConfig kDriveMotorOutputConfig = null;
    public final voltageConfig kDriveVoltageConfig = null;
    public final currentConfig kAzimuthCurrentConfig = null;
    public final motorOutputConfig kAzimuthMotorOutputConfig = null;
    public final voltageConfig kAzimuthVoltageConfig = null;

    // Declare team specific constants here.
    public final moduleOffsets kFLModuleOffsets;
    public final moduleIDs kFLModuleIDs;
    public final moduleOffsets kFRModuleOffsets;
    public final moduleIDs kFRModuleIDs;
    public final moduleOffsets kBLModuleOffsets;
    public final moduleIDs kBLModuleIDs;
    public final moduleOffsets kBRModuleOffsets;
    public final moduleIDs kBRModuleIDs;
    public final softLimits kSoftLimits;
    public final PIDController kDrivePIDController;
    public final PIDController kAzimuthPIDController;
    public final feedforwardGains kDriveGains;
    public final feedforwardGains kAzimuthGains;
    public final double kJoystickDeadzone = 0.1d;

    private DriveConstants() {
        kFLModuleIDs = new moduleIDs(0, 0, 0);
        kFRModuleIDs = new moduleIDs(0, 0, 0);
        kBLModuleIDs = new moduleIDs(0, 0, 0);
        kBRModuleIDs = new moduleIDs(0, 0, 0);

        switch (RobotConstants.getInstance().kTeamNumber) {
            case 5411:
                // Assign team specific constants.
                kFLModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d);
                kFRModuleOffsets = new moduleOffsets(new Translation2d(0.5d, 0.5d), 0.0d);
                kBLModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, -0.5d), 0.0d);
                kBRModuleOffsets = new moduleOffsets(new Translation2d(0.5d, -0.5d), 0.0d);
                kDrivePIDController = new PIDController(0.05d, 0.0d, 0.025);
                kAzimuthPIDController = new PIDController(0.05d, 0.0d, 0.025d);
                kSoftLimits = new softLimits(0.5d, 4.5d);
                kDriveGains = new feedforwardGains(0.0d, 0.0d, 0.0d);
                kAzimuthGains = new feedforwardGains(0.0d, 0.0d, 0.0d);
                break;
            case 9105:
                // Assign team specific constants.
                kFLModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d);
                kFRModuleOffsets = new moduleOffsets(new Translation2d(0.5d, 0.5d), 0.0d);
                kBLModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, -0.5d), 0.0d);
                kBRModuleOffsets = new moduleOffsets(new Translation2d(0.5d, -0.5d), 0.0d);
                kDrivePIDController = new PIDController(0.05d, 0.0d, 0.025);
                kAzimuthPIDController = new PIDController(0.05d, 0.0d, 0.025d);
                kSoftLimits = new softLimits(0.5d, 4.5d);
                kDriveGains = new feedforwardGains(0.0d, 0.0d, 0.0d);
                kAzimuthGains = new feedforwardGains(0.0d, 0.0d, 0.0d);
                break;
            case 9492:
                // Assign team specific constants.
                kFLModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d);
                kFRModuleOffsets = new moduleOffsets(new Translation2d(0.5d, 0.5d), 0.0d);
                kBLModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, -0.5d), 0.0d);
                kBRModuleOffsets = new moduleOffsets(new Translation2d(0.5d, -0.5d), 0.0d);
                kDrivePIDController = new PIDController(0.05d, 0.0d, 0.025);
                kAzimuthPIDController = new PIDController(0.05d, 0.0d, 0.025d);
                kSoftLimits = new softLimits(0.5d, 4.5d);
                kDriveGains = new feedforwardGains(0.0d, 0.0d, 0.0d);
                kAzimuthGains = new feedforwardGains(0.0d, 0.0d, 0.0d);
                break;
            case 0:
                // Assign sim constants
                kFLModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d);
                kFRModuleOffsets = new moduleOffsets(new Translation2d(0.5d, 0.5d), 0.0d);
                kBLModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, -0.5d), 0.0d);
                kBRModuleOffsets = new moduleOffsets(new Translation2d(0.5d, -0.5d), 0.0d);
                kDrivePIDController = new PIDController(0.25d, 0.0d, 0.0d);
                kAzimuthPIDController = new PIDController(20.0d, 0.0d, 0.0d);
                kSoftLimits = new softLimits(1.0d, 4.0d);
                kDriveGains = new feedforwardGains(0.0d, 0.0d, 0.0d);
                kAzimuthGains = new feedforwardGains(0.0d, 0.0d, 0.0d);
                break;
            default:
                // Assign fallback constants.
                kFLModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, 0.5d), 0.0d);
                kFRModuleOffsets = new moduleOffsets(new Translation2d(0.5d, 0.5d), 0.0d);
                kBLModuleOffsets = new moduleOffsets(new Translation2d(-0.5d, -0.5d), 0.0d);
                kBRModuleOffsets = new moduleOffsets(new Translation2d(0.5d, -0.5d), 0.0d);
                kDrivePIDController = new PIDController(0.05d, 0.0d, 0.0);
                kAzimuthPIDController = new PIDController(0.05d, 0.0d, 0.025d);
                kSoftLimits = new softLimits(0.05d, 4.5d);
                kDriveGains = new feedforwardGains(0.0d, 0.0d, 0.0d);
                kAzimuthGains = new feedforwardGains(0.0d, 0.0d, 0.0d);
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
