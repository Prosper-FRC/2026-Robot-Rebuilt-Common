package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
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

public class Drive extends SubsystemBase {
    public static final Drive NoOp = new Drive(
        new SwerveModuleIO() {}, 
        new SwerveModuleIO() {}, 
        new SwerveModuleIO() {}, 
        new SwerveModuleIO() {}, 
        new GyroIO() {}
    );

    public enum DriveState {
        TELEOP,
        AUTON,
        SYSID
    }
    public DriveState robotState = DriveState.TELEOP;

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

    private final TeleopController kTeleopController;
    private final HolonomicController kHolonomicController;

    public Drive(
        SwerveModuleIO FLModule,
        SwerveModuleIO FRModule,
        SwerveModuleIO BLModule,
        SwerveModuleIO BRModule,
        GyroIO gyro
    ) {
        kTeleopController = new TeleopController();
        kHolonomicController = new HolonomicController();

        kRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(0.75d), Volts.of(6d), Second.of(8.0), // Default values
            (sysidState) -> Logger.recordOutput("Drive/SysIdState", sysidState.toString())),
            new SysIdRoutine.Mechanism((voltage) -> this.applySysIdVoltage(voltage.in(Volts)), 
            null, // AK will be logging the values here.
            this)
        );

        kModules[0] = FLModule;
        kModules[1] = FRModule;
        kModules[2] = BLModule;
        kModules[3] = BRModule;
        kGyro = gyro;

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
                    RobotConstants.DriveConstants().kDriveSoftLimits.maxLinearVelocityMPS(),
                    RobotConstants.DriveConstants().kDriveSoftLimits.maxAngularVelocityRPS()
                )
            )
            .withSwerveHardware(
                new SwerveHardware(
                    RobotConstants.DriveConstants().kModuleHardware.wheelRadiusMeters(), 
                    RobotConstants.DriveConstants().kModuleHardware.driveSideLengthsMeters()
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
    private void setDriveState(DriveState state) {
        robotState = state;
    }
    private void setSwerveModuleOutputs(SwerveModuleState[] moduleStates) {
        for(int i = 0; i < 4; ++i) {
            kModules[i].setDriveSpeedWithVoltage(metersToRotations(moduleStates[i].speedMetersPerSecond));
            kModules[i].setAzimuthPositionWithVoltage(moduleStates[i].angle.getRotations());
        }
    }

    /********** GETTER METHODS **********/

    private Rotation2d getGyroReading() {
        return Rotation2d.fromRotations(kGyroInputs.gyroPositionRotations);
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

    /********** CONTROL METHODS **********/
    public void stopDriveMotors() {
        for(var module : kModules) {
            module.stopDriveMotor();
        }
    }

    public void stopAzimuthMotors() {
        for(var module : kModules) {
            module.stopAzimuthMotor();
        }
    }

    /********** COMMAND METHODS **********/
    public void supplyControllerInputs(DoubleSupplier inputX, DoubleSupplier inputY, DoubleSupplier inputOmega) {
        kTeleopController.supplyControllerInputs(
            inputX, 
            inputY, 
            inputOmega
        );
    }

    public Command setDriveStateCommand(DriveState state) {
        return new RunCommand(() -> setDriveState(state), this);
    }

    public Command resetGyroCommand() {
        return new InstantCommand(() -> kGyro.resetGyro(), this);
    }

    public Command followTrajectoryCommand(Pose2d trajectoryPoint) {
        return new InstantCommand(() -> kHolonomicController.setDesiredFieldPose(trajectoryPoint), this);
    }

    public Command stopDriveMotorsCommand() {
        return new InstantCommand(() -> stopDriveMotors(), this);
    }

    public Command stopAzimuthMotorsCommand() {
        return new InstantCommand(() -> stopAzimuthMotors(), this);
    }

    /******** SYSID ********/
    private final SysIdRoutine kRoutine;

    public Command getSysIdCommand() {
        // Create Dynamic tests
        Command dynamicForward = kRoutine.dynamic(SysIdRoutine.Direction.kForward);
        Command dynamicReverse = kRoutine.dynamic(SysIdRoutine.Direction.kReverse);

        // Create quasistatic tests
        Command quasistaticForward = kRoutine.quasistatic(SysIdRoutine.Direction.kForward);
        Command quasistaticReverse = kRoutine.quasistatic(SysIdRoutine.Direction.kReverse);

        // Schedule the tests.
        return new SequentialCommandGroup(
            lockAzimuthMotorsCommand().andThen(
                dynamicForward.andThen(new WaitCommand(0.2d).alongWith(stopDriveMotorsCommand())).andThen(dynamicReverse).andThen(new WaitCommand(0.2d).alongWith(stopDriveMotorsCommand()))
                .andThen(quasistaticForward).andThen(new WaitCommand(0.2d).alongWith(stopDriveMotorsCommand())).andThen(quasistaticReverse).andThen(new WaitCommand(0.2d).alongWith(stopDriveMotorsCommand()))
            )
        );
    }

    public Command lockAzimuthMotorsCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                    for(var module : kModules) {
                        module.setAzimuthPositionWithVoltage(0.0d);
                    }
                }
            ),
            new WaitCommand(0.25d)
        );
    }

    // Used to direct set the voltages of each drive motor for PID.
    // NOTE: Before running a SysId test it is smart to ensure all Azimuth motors are facing relatively forwards.
    private void applySysIdVoltage(double driveVolts) {
        for(var module : kModules) {
            // Set drive goal to SysId given voltage.
            module.setDriveMotorVoltage(driveVolts);
            // Set azimuth goal to zero degrees (To hold the azimuths in place).
            module.setAzimuthPositionWithVoltage(0.0d);
        }
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

        kGyro.updateGyro(Units.radiansToRotations(realSpeeds.omegaRadiansPerSecond) * RobotConstants.Instance().kTimestep);

        odometryPose = kSwerveOdometry.update(getGyroReading(), getModulePositions());

        switch (robotState) {
            case TELEOP:
                desiredSpeeds = kTeleopController.getDesiredSpeeds();
                stateUpdateTeleop();
                break;
            case AUTON:
                desiredSpeeds = kHolonomicController.calculatePositionSetpoint(kSwerveOdometry.getPoseMeters(), new ChassisSpeeds());
                stateUpdateAuton();
                break;
            default:
                break;
        }
    }

    /********** STATE HANDLING **********/
    public void stateUpdateTeleop() {
        ChassisSpeeds robotRelativeDesiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, getGyroReading());

        targetStates = kSetpointGenerator.generateSetpoint(robotRelativeDesiredSpeeds, actualStates);

        setSwerveModuleOutputs(targetStates);
    }

    public void stateUpdateAuton() {
        ChassisSpeeds robotRelativeDesiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, getGyroReading());

        targetStates = kSetpointGenerator.generateSetpoint(robotRelativeDesiredSpeeds, actualStates);

        setSwerveModuleOutputs(targetStates);
    }
}
