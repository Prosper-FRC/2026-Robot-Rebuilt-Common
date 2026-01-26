package frc.robot.Subsystems.Drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

public class Drive extends SubsystemBase {
    // State enum
    public enum driveState {
        Teleop
    }

    // Initialization of modules
    private final ModuleIO[] kModules = new ModuleIO[4];
    private final moduleInputsAutoLogged[] kModuleInputs = new moduleInputsAutoLogged[] {
        new moduleInputsAutoLogged(), 
        new moduleInputsAutoLogged(),
        new moduleInputsAutoLogged(),
        new moduleInputsAutoLogged()
    };
    private final GyroIO kGyro;
    private final gyroInputsAutoLogged kGyroInputs = new gyroInputsAutoLogged();

    private DoubleSupplier joystickX = () -> 0.0d;
    private DoubleSupplier joystickY = () -> 0.0d;
    private DoubleSupplier joystickTheta = () -> 0.0d;
    private final SlewRateLimiter kXRateLimiter = new SlewRateLimiter(7.0d);
    private final SlewRateLimiter kYRateLimiter = new SlewRateLimiter(7.0d);

    private final SwerveDriveKinematics kKinematicsProcessor;

    private driveState state = driveState.Teleop;
    
    @AutoLogOutput(key = "Drive/OdometryPose")
    private Pose2d kOdometryPose = new Pose2d();
    private final SwerveDriveOdometry kOdometry;
    private ChassisSpeeds kGoalSpeeds;

    // Modules should be inputted in this order: FL, FR, BL, BR.
    public Drive(ModuleIO[] modules, GyroIO gyro) {
        for (int i = 0; i < 4; ++i) {
            kModules[i] = modules[i];
        }
        kGyro = gyro;

        kKinematicsProcessor = new SwerveDriveKinematics(
            DriveConstants.getInstance().kFLModuleOffsets.poseOffset(),
            DriveConstants.getInstance().kFRModuleOffsets.poseOffset(),
            DriveConstants.getInstance().kBLModuleOffsets.poseOffset(),
            DriveConstants.getInstance().kBRModuleOffsets.poseOffset()
        );
        kOdometry = new SwerveDriveOdometry(
            kKinematicsProcessor, 
            new Rotation2d(Units.rotationsToRadians(kGyro.getYawAngleRotations())),
            new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}
        );
        kGoalSpeeds = new ChassisSpeeds();
    }

    @Override
    public void periodic() {
        for(int i = 0; i < 4; ++i) {
            kModules[i].updateInputs(kModuleInputs[i]);
            Logger.processInputs("Drive/Module" + String.valueOf(i), kModuleInputs[i]);
        }
        kGyro.updateInputs(kGyroInputs);
        Logger.processInputs("Drive/Gyro", kGyroInputs);

        switch (state) {
            case Teleop:
                teleopState();
                break;
            default:
                break;
        }
    }

    private double metersToRotations(double meters) {
        return meters / (Math.PI * 2) / DriveConstants.getInstance().kWheelRadiusMeters;
    }
    private double rotationsToMeters(double rotations) {
        return rotations * (Math.PI * 2) * DriveConstants.getInstance().kWheelRadiusMeters;
    }

    private double[] processJoystickInputs(double x, double y, double omega) {
        double processedX = Math.pow(x, 2);
        double processedY = Math.pow(y, 2);
        double processedOmega = Math.pow(omega, 2);

        processedX = MathUtil.applyDeadband(processedX, DriveConstants.getInstance().kJoystickDeadzone);
        processedY = MathUtil.applyDeadband(processedY, DriveConstants.getInstance().kJoystickDeadzone);
        processedOmega = MathUtil.applyDeadband(processedOmega, DriveConstants.getInstance().kJoystickDeadzone);

        processedX *= Math.signum(x);
        processedY *= Math.signum(y);

        processedX = kXRateLimiter.calculate(processedX);
        processedY = kYRateLimiter.calculate(processedY);
        
        processedOmega *= Math.signum(omega);

        processedX *= DriveConstants.getInstance().kSoftLimits.maximumLinearVelocityMPS();
        processedY *= DriveConstants.getInstance().kSoftLimits.maximumLinearVelocityMPS();
        processedOmega *= Units.rotationsToRadians(DriveConstants.getInstance().kSoftLimits.maximumAngularVelocityRotations());

        return new double[]{processedX, processedY, processedOmega};
    }

    //////////// STATE UPDATES ////////////
    private void teleopState() {
        double[] processedInputs = processJoystickInputs(joystickX.getAsDouble(), joystickY.getAsDouble(), joystickTheta.getAsDouble());

        kGoalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(processedInputs[1], processedInputs[0], -processedInputs[2], new Rotation2d(Units.rotationsToRadians(kGyro.getYawAngleRotations())));
        SwerveModuleState[] desaturateStates = kKinematicsProcessor.toSwerveModuleStates(kGoalSpeeds);
        // Desaturate the wheels before discretizing them.
        for(int i = 0; i < 4; ++i) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desaturateStates, DriveConstants.getInstance().kSoftLimits.maximumLinearVelocityMPS());
        }
        // Reconvert these desaturated speeds to a chassis to be discretized
        kGoalSpeeds = kKinematicsProcessor.toChassisSpeeds(desaturateStates);
        kGoalSpeeds = ChassisSpeeds.discretize(kGoalSpeeds, RobotConstants.getInstance().kTimestep);
        SwerveModuleState[] states = kKinematicsProcessor.toSwerveModuleStates(kGoalSpeeds);
        updateGyroOdometry();
            
        for (int i = 0; i < 4; ++i) {
            states[i].optimize(new Rotation2d(Units.rotationsToRadians(kModuleInputs[i].azimuthPositionRotations)));
            kModules[i].setAzimuthRotations(states[i].angle.getRotations());
            kModules[i].setDriveRPS(metersToRotations(states[i].speedMetersPerSecond));
        }
        kOdometry.update(new Rotation2d(Units.rotationsToRadians(kGyro.getYawAngleRotations())),
            new SwerveModulePosition[]
            {
                new SwerveModulePosition(rotationsToMeters(kModuleInputs[0].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[0].azimuthPositionRotations))),
                new SwerveModulePosition(rotationsToMeters(kModuleInputs[1].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[1].azimuthPositionRotations))),
                new SwerveModulePosition(rotationsToMeters(kModuleInputs[2].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[2].azimuthPositionRotations))),
                new SwerveModulePosition(rotationsToMeters(kModuleInputs[3].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[3].azimuthPositionRotations)))
            }
        );
        kOdometryPose = kOdometry.getPoseMeters();
    }

    private void updateGyroOdometry() {
        SwerveModuleState[] currentState = new SwerveModuleState[4];
        for(int i = 0; i < 4; ++i) {
            currentState[i] = new SwerveModuleState(rotationsToMeters(kModuleInputs[i].driveVelocityRPS), new Rotation2d(Units.rotationsToRadians(kModuleInputs[i].azimuthPositionRotations)));
        }

        ChassisSpeeds currentSpeeds = kKinematicsProcessor.toChassisSpeeds(currentState);
        kGyro.updateYaw(Units.radiansToRotations(currentSpeeds.omegaRadiansPerSecond), RobotConstants.getInstance().kTimestep);
    }

    // Public methods
    public void assignJoysticks(DoubleSupplier stickX, DoubleSupplier stickY, DoubleSupplier stickTheta) {
        joystickX = stickX;
        joystickY = stickY;
        joystickTheta = stickTheta;
    }

    public void setDriveState(driveState stateToSet) {
        state = stateToSet;
    }
}
