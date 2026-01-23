package frc.robot.Subsystems.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final Module[] kModules = new Module[4];
    private final Gyro kGyro;

    private DoubleSupplier joystickX = () -> 0.0d;
    private DoubleSupplier joystickY = () -> 0.0d;
    private DoubleSupplier joystickTheta = () -> 0.0d;

    private final SwerveDriveKinematics kKinematicsProcessor;
    private final SwerveDriveOdometry kOdometry;
    private final ChassisSpeeds kGoalSpeeds;

    // Modules should be inputted in this order: FL, FR, BL, BR.
    public Drive(Module[] modules, Gyro gyro) {
        for (int i = 0; i < 4; ++i) {
            kModules[i] = modules[i];
        }
        kGyro = gyro;

        kKinematicsProcessor = new SwerveDriveKinematics(
            DriveConstants.getInstance().kModuleOffsets.FLPoseOffset(),
            DriveConstants.getInstance().kModuleOffsets.FRPoseOffset(),
            DriveConstants.getInstance().kModuleOffsets.BLPoseOffset(),
            DriveConstants.getInstance().kModuleOffsets.BRPoseOffset()
        );
        kOdometry = new SwerveDriveOdometry(
            kKinematicsProcessor, 
            new Rotation2d(kGyro.getRotations().getZ() * (2 * Math.PI)), 
            new SwerveModulePosition[]
            {
                kModules[0].getAsSwerveModulePosition(), 
                kModules[1].getAsSwerveModulePosition(),
                kModules[2].getAsSwerveModulePosition(),
                kModules[3].getAsSwerveModulePosition()
            }
        );
        kGoalSpeeds = new ChassisSpeeds();
    }

    public void assignJoysticks(DoubleSupplier stickX, DoubleSupplier stickY, DoubleSupplier stickTheta) {
        joystickX = stickX;
        joystickY = stickY;
        joystickTheta = stickTheta;
    }

    @Override
    public void periodic() {
        ChassisSpeeds.fromFieldRelativeSpeeds(joystickX.getAsDouble(), joystickY.getAsDouble(), joystickTheta.getAsDouble() * (2 * Math.PI), new Rotation2d(kGyro.getRotations().getMeasureZ()));
        SwerveModuleState[] states = kKinematicsProcessor.toSwerveModuleStates(kGoalSpeeds);
        
        for (int i = 0; i < 4; ++i) {
            kModules[i].setAzimuthRotations(states[i].angle.getRotations());
            kModules[i].setDriveMPS(states[i].speedMetersPerSecond * DriveConstants.getInstance().kSoftLimits.maximumLinearVelocityMPS());
        }

        kOdometry.update(new Rotation2d(kGyro.getRotations().getZ()),
            new SwerveModulePosition[]
            {
                kModules[0].getAsSwerveModulePosition(), 
                kModules[1].getAsSwerveModulePosition(),
                kModules[2].getAsSwerveModulePosition(),
                kModules[3].getAsSwerveModulePosition()
            });
    }
}
