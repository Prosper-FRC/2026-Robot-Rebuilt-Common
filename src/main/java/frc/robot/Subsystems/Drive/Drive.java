package frc.robot.Subsystems.Drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.Controllers.TeleopController;

public class Drive extends SubsystemBase {
    public enum driveState {
        TELEOP,
        TELEOP_SNIPER
    }
    private driveState state = driveState.TELEOP;

    // Create IO layers
    private final ModuleIO[] kModules;
    private final GyroIO kGyro;

    // Create inputs
    private final moduleInputsAutoLogged[] kModuleInputs = new moduleInputsAutoLogged[] {
        new moduleInputsAutoLogged(),
        new moduleInputsAutoLogged(),
        new moduleInputsAutoLogged(),
        new moduleInputsAutoLogged()
    };

    private final gyroInputsAutoLogged kGyroInputs = new gyroInputsAutoLogged();

    // Initialize swerve related tools
    private final SwerveDriveKinematics kKinematics;
    private final SwerveDriveOdometry kOdometry;

    @AutoLogOutput(key = "Drive/Swerve/Speeds")
    private ChassisSpeeds desiredSpeeds;

    @AutoLogOutput(key = "Drive/OdometryPose")
    private Pose2d odometryPose = new Pose2d();

    @AutoLogOutput(key = "Drive/Swerve/States")
    private SwerveModuleState[] states;

    // For teleop control
    private final TeleopController kTeleopController = new TeleopController();
    
    public Drive(ModuleIO moduleFR, ModuleIO moduleFL, ModuleIO moduleBR, ModuleIO moduleBL, GyroIO gyro) {
        kModules = new ModuleIO[] {
            moduleFR,
            moduleFL,
            moduleBR,
            moduleBL
        };
        kGyro = gyro;

        kKinematics = new SwerveDriveKinematics(
            RobotConstants.DriveConstants().kFRModuleOffsets.translationalOffset(),
            RobotConstants.DriveConstants().kFLModuleOffsets.translationalOffset(),
            RobotConstants.DriveConstants().kBRModuleOffsets.translationalOffset(),
            RobotConstants.DriveConstants().kBLModuleOffsets.translationalOffset()
        );

        kOdometry = new SwerveDriveOdometry(kKinematics, 
            new Rotation2d(0.0d), 
            getModulePositions()
        );
    }

    public void supplyControllerInputs(DoubleSupplier xInputs, DoubleSupplier yInput, DoubleSupplier angleInput) {
        kTeleopController.supplyControllerInputs(xInputs, yInput, angleInput);
    }

    public void setDriveState(driveState driveState) { state = driveState; }

    public driveState getDriveState() { return state; }
    
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(rotationsToMeters(kModuleInputs[0].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[0].azimuthPositionRotations))),
            new SwerveModulePosition(rotationsToMeters(kModuleInputs[1].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[1].azimuthPositionRotations))),
            new SwerveModulePosition(rotationsToMeters(kModuleInputs[2].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[2].azimuthPositionRotations))),
            new SwerveModulePosition(rotationsToMeters(kModuleInputs[3].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[3].azimuthPositionRotations)))
        };
    }

    public void optimizeModules(SwerveModuleState[] states) {
        for(int i = 0; i < states.length; ++i) {
        }
    }

    public double rotationsToMeters(double rotations) {
        return Units.rotationsToRadians(rotations) * RobotConstants.DriveConstants().kModuleHardLimits.wheelRadiusMeters();
    }
    
    public double metersToRotations(double meters) {
        return meters / ((2 * Math.PI) * RobotConstants.DriveConstants().kModuleHardLimits.wheelRadiusMeters());
    }

    @Override
    public void periodic() {
        // Update inputs for IO layers.
        for(int i = 0; i < kModuleInputs.length; ++i) {
            kModules[i].updateInputs(kModuleInputs[i]);
        }
        kGyro.updateInputs(kGyroInputs);

        // Update AK Logging.
        Logger.processInputs("Drive/ModuleFR", kModuleInputs[0]);
        Logger.processInputs("Drive/ModuleFL", kModuleInputs[1]);
        Logger.processInputs("Drive/ModuleBR", kModuleInputs[2]);
        Logger.processInputs("Drive/ModuleBL", kModuleInputs[3]);
        Logger.processInputs("Drive/Gyro", kGyroInputs);

        // Update Odometry.
        odometryPose = kOdometry.update(new Rotation2d(Units.rotationsToRadians(kGyroInputs.yawRotations)), getModulePositions());

        // Internal State Handling.
        switch(state) {
            case TELEOP:
                // Compute chassis speeds.
                desiredSpeeds = kTeleopController.getDesiredSpeeds(false);
                break;
            case TELEOP_SNIPER:
                desiredSpeeds = kTeleopController.getDesiredSpeeds(true);
                break;
            default:
                break;
        }
        // Discretized robot framed chassis speeds.
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, new Rotation2d(Units.rotationsToRadians(kGyroInputs.yawRotations)));
        ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, RobotConstants.Instance().kTimestep);

        // Perform IK to get each indiviual module's goal setpoint and then desaturate to cap the speed.
        SwerveModuleState[] moduleStates = kKinematics.toSwerveModuleStates(discretizedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, RobotConstants.DriveConstants().kModuleSoftLimits.maxLinearVelocityMPS());

        // Optimize the modules so they never rotate more than 90 degrees.
        optimizeModules(moduleStates);
        states = moduleStates;

        // Apply the modules goals to the actual motor.
        for(int i = 0; i < kModules.length; ++i) {
            kModules[i].setDriveRPS(metersToRotations(moduleStates[i].speedMetersPerSecond));
            kModules[i].setAzimuthRotations(moduleStates[i].angle.getRotations());
        }
    }
}
