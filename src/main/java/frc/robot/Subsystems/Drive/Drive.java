package frc.robot.Subsystems.Drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.Controllers.HeadingController;
import frc.robot.Subsystems.Drive.Controllers.HolonomicController;
import frc.robot.Subsystems.Drive.Controllers.TeleopController;
import frc.robot.Subsystems.Drive.Gyro.GyroIO;
import frc.robot.Subsystems.Drive.Gyro.GyroInputsAutoLogged;
import frc.robot.Subsystems.Drive.SwerveModule.SwerveModuleIO;
import frc.robot.Subsystems.Drive.SwerveModule.SwerveModuleInputsAutoLogged;
import frc.robot.Subsystems.Drive.SwerveSetpointGenerator.SetpointGenerator;
import frc.robot.Subsystems.Drive.SwerveSetpointGenerator.SwerveConfiguration;
import frc.robot.Subsystems.Drive.SwerveSetpointGenerator.SwerveConfiguration.ChassisTranslations;
import frc.robot.Subsystems.Drive.SwerveSetpointGenerator.SwerveConfiguration.ModuleConstraints;
import frc.robot.Subsystems.Drive.SwerveSetpointGenerator.SwerveConfiguration.SwerveHardware;
import frc.robot.Subsystems.Vision.Vision;

public class Drive extends SubsystemBase {
    public enum RobotState {
        TELEOP,
        AUTON,
        HUB_HEADING_ALIGN
    }
    public RobotState robotState = RobotState.TELEOP;

    // Setpoint generator setup
    private final SwerveConfiguration kConfiguration;
    private final SetpointGenerator kSetpointGenerator;
    private final SwerveDriveKinematics kSwerveKinematics;
    private final SwerveDriveOdometry kSwerveOdometry;

    // Hardware
    private final SwerveModuleIO[] kModules = new SwerveModuleIO[4];
    private final SwerveModuleInputsAutoLogged[] kModuleInputs = new SwerveModuleInputsAutoLogged[] {
        new SwerveModuleInputsAutoLogged(),
        new SwerveModuleInputsAutoLogged(),
        new SwerveModuleInputsAutoLogged(),
        new SwerveModuleInputsAutoLogged()
    };

    private final GyroIO kGyro;
    private final GyroInputsAutoLogged kGyroInputs = new GyroInputsAutoLogged();
    
    private final Vision kVision;

    // Logging
    @AutoLogOutput(key = "Drive/Odometry/OdometryPose")
    private Pose2d odometryPose = new Pose2d();

    @AutoLogOutput(key = "Drive/Speeds/RealSpeeds")
    private ChassisSpeeds realSpeeds = new ChassisSpeeds();
    @AutoLogOutput(key = "Drive/Speeds/DesiredSpeeds")
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    @AutoLogOutput(key = "Drive/ModuleStates/Real")
    private SwerveModuleState[] actualStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    @AutoLogOutput(key = "Drive/ModuleStates/Target")
    private SwerveModuleState[] targetStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private final HeadingController kHeadingController;
    private final TeleopController kTeleopController;

    public Drive(
        SwerveModuleIO FLModule,
        SwerveModuleIO FRModule,
        SwerveModuleIO BLModule,
        SwerveModuleIO BRModule,
        GyroIO gyro,
        Vision vision
    ) {
        kTeleopController = new TeleopController();
        kHeadingController = new HeadingController();

        kModules[0] = FLModule;
        kModules[1] = FRModule;
        kModules[2] = BLModule;
        kModules[3] = BRModule;
        kGyro = gyro;
        kVision = vision;

        double trackDistance = RobotConstants.DriveConstants().kModuleHardware.driveSideLengthsMeters() / 2.0d;
        kConfiguration = new SwerveConfiguration()
            .withChassisTranslations(
                new ChassisTranslations(
                    new Translation2d(trackDistance, trackDistance), 
                    new Translation2d(trackDistance, -trackDistance), 
                    new Translation2d(-trackDistance, trackDistance), 
                    new Translation2d(-trackDistance, -trackDistance))
            )
            .withModuleConstraints(
                new ModuleConstraints(
                    4.5d, 
                    1.0d
                )
            )
            .withSwerveHardware(
                new SwerveHardware(
                    Units.inchesToMeters(2.0d), 
                    Units.inchesToMeters(26.5d)
                )
            );
        kSetpointGenerator = new SetpointGenerator(kConfiguration);
        kSwerveKinematics = kSetpointGenerator.getKinematics();
        kSwerveOdometry = new SwerveDriveOdometry(
            kSwerveKinematics, 
            Rotation2d.fromRotations(0.0d), 
            getModulePositions());
    }

    /********** HELPER METHODS **********/

    private double rotationsToMeters(double rotations) {
        return (rotations * kConfiguration.swerveHardware.wheelRadiusMeters()) * (2 * Math.PI);
    }
    private double metersToRotations(double meters) {
        return (meters / kConfiguration.swerveHardware.wheelRadiusMeters()) / (2 * Math.PI);
    }
    private void setDriveState(RobotState state) {
        robotState = state;
    }

    /********** GETTER METHODS **********/

    // Get vision pose if it is valid, and return odometry pose if not. 
    private Pose2d getPoseEstimate(Pose2d odomPose) {
        if (RobotBase.isReal()) {
            if (kVision.getValidPose().getX() != Double.MAX_VALUE) {
                return kVision.getValidPose();
            } else {
                return odomPose;
            }
        } else {
            return odomPose;
        }
    }

    private Rotation2d getGyroReading() {
        return Rotation2d.fromRotations(kGyroInputs.gyroPositionRotations);
    }

    private Rotation2d getRobotAngle() {
        return kSwerveOdometry.getPoseMeters().getRotation();     
    }

    public double[] getDriveMotorPositionsRotations() {
        return new double[] {
            rotationsToMeters(kModuleInputs[0].driveModulePositionRotations),
            rotationsToMeters(kModuleInputs[1].driveModulePositionRotations),
            rotationsToMeters(kModuleInputs[2].driveModulePositionRotations),
            rotationsToMeters(kModuleInputs[3].driveModulePositionRotations)
        };
    }

    public double[] getDriveMotorSpeedsMetersPerSecond() {
        return new double[] {
            rotationsToMeters(kModuleInputs[0].driveModuleSpeedRotationsPerSecond),
            rotationsToMeters(kModuleInputs[1].driveModuleSpeedRotationsPerSecond),
            rotationsToMeters(kModuleInputs[2].driveModuleSpeedRotationsPerSecond),
            rotationsToMeters(kModuleInputs[3].driveModuleSpeedRotationsPerSecond)
        };
    }

    public double[] getAzimuthMotorPositionsRotations() {
        return new double[] {
            kModuleInputs[0].azimuthModulePositionRotations,
            kModuleInputs[1].azimuthModulePositionRotations,
            kModuleInputs[2].azimuthModulePositionRotations,
            kModuleInputs[3].azimuthModulePositionRotations
        };
    }

    private SwerveModulePosition[] getModulePositions() {
        double[] driveSpeeds = getDriveMotorPositionsRotations();
        double[] azimuthAngles = getAzimuthMotorPositionsRotations();
        return new SwerveModulePosition[] {
            new SwerveModulePosition(driveSpeeds[0], Rotation2d.fromRotations(azimuthAngles[0])),
            new SwerveModulePosition(driveSpeeds[1], Rotation2d.fromRotations(azimuthAngles[1])),
            new SwerveModulePosition(driveSpeeds[2], Rotation2d.fromRotations(azimuthAngles[2])),
            new SwerveModulePosition(driveSpeeds[3], Rotation2d.fromRotations(azimuthAngles[3]))
        };
    }

    /********** COMMAND METHODS **********/
    public void supplyControllerInputs(DoubleSupplier inputX, DoubleSupplier inputY, DoubleSupplier inputOmega) {
        kTeleopController.supplyControllerInputs(
            inputX, 
            inputY, 
            inputOmega
        );
    }

    /********** COMMAND METHODS **********/
    public Command setDriveStateCommand(RobotState state) {
        return new RunCommand(() -> setDriveState(state), this);
    }

    /********** PERIODIC **********/
    @Override
    public void periodic() {
        for (int i = 0; i < 4; ++i) {
            kModules[i].updateInputs(kModuleInputs[i]);
        }
        kGyro.updateInputs(kGyroInputs);
        Logger.processInputs("Drive/ModuleFL", kModuleInputs[0]);
        Logger.processInputs("Drive/ModuleFR", kModuleInputs[1]);
        Logger.processInputs("Drive/ModuleBL", kModuleInputs[2]);
        Logger.processInputs("Drive/ModuleBR", kModuleInputs[3]);
        Logger.processInputs("Drive/Gyro", kGyroInputs);

        double[] driveModules = getDriveMotorSpeedsMetersPerSecond();
        double[] azimuthModules = getAzimuthMotorPositionsRotations();
        actualStates = new SwerveModuleState[] {
            new SwerveModuleState(driveModules[0], Rotation2d.fromRotations(azimuthModules[0])),
            new SwerveModuleState(driveModules[1], Rotation2d.fromRotations(azimuthModules[1])),
            new SwerveModuleState(driveModules[2], Rotation2d.fromRotations(azimuthModules[2])),
            new SwerveModuleState(driveModules[3], Rotation2d.fromRotations(azimuthModules[3]))
        };
        realSpeeds = kSwerveKinematics.toChassisSpeeds(actualStates);

        odometryPose = kSwerveOdometry.update(odometryPose.getRotation().plus(Rotation2d.fromRadians(realSpeeds.omegaRadiansPerSecond).times(RobotConstants.Instance().kTimestep)), getModulePositions());

        // Supply yaw to Vision to use MegaTag2 Localization
        double yaw = (getRobotAngle().getDegrees());
        kVision.setYaw(yaw);

        // ChassisSpeeds teleopSpeeds = kTeleopController.
        switch (robotState) {
            case TELEOP:
                desiredSpeeds = kTeleopController.getDesiredSpeeds();
                stateUpdateTeleop();
                break;
            case AUTON:
                break;
            case HUB_HEADING_ALIGN:
                // Get current rotation/yaw from vision or get it from odometry 
                Rotation2d currentRotation;
                if (kVision.getValidPose().getRotation() == Rotation2d.fromDegrees(Double.MAX_VALUE)) {
                    currentRotation = getRobotAngle();
                } else {
                    currentRotation = kVision.getValidPose().getRotation();
                }
                // Find angular velocity needed to rotate to hub by using current yaw as a starting point
                // and kVision.getAlignment as the end point
                double angularVelocity = kHeadingController.AimDownSights(currentRotation, kVision.getAlignment());
                desiredSpeeds = new ChassisSpeeds(realSpeeds.vxMetersPerSecond, realSpeeds.vyMetersPerSecond, angularVelocity);
                stateUpdateTeleop();
                break;
            default:
                break;
        }
    }
    
    /********** STATE HANDLING **********/
    public void stateUpdateTeleop() {
        ChassisSpeeds robotRelativeDesiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, getRobotAngle());

        targetStates = kSetpointGenerator.generateSetpoint(robotRelativeDesiredSpeeds, actualStates);

        for(int i = 0; i < 4; ++i) {
            kModules[i].setDriveSpeedWithVoltage(metersToRotations(targetStates[i].speedMetersPerSecond));
            kModules[i].setAzimuthPositionWithVoltage(targetStates[i].angle.getRotations());
        }
    }
}
