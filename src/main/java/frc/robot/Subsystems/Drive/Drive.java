package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.Controllers.HeadingController;
import frc.robot.Subsystems.Drive.Controllers.HolonomicController;
import frc.robot.Subsystems.Drive.Controllers.TeleopController;

public class Drive extends SubsystemBase {
    public enum driveState {
        DISABLED,
        TELEOP,
        TELEOP_SNIPER,
        AUTON,
        SYSID
    }

    @AutoLogOutput(key = "Drive/DriveState")
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

    @AutoLogOutput(key = "Drive/Swerve/Speeds")
    private ChassisSpeeds desiredSpeeds;

    @AutoLogOutput(key = "Drive/OdometryPose")
    private Pose2d odometryPose = new Pose2d();

    @AutoLogOutput(key = "Drive/PoseEstimator")
    private Pose2d poseEstimator = new Pose2d();

    @AutoLogOutput(key = "Drive/Swerve/States")
    private SwerveModuleState[] states;

    @AutoLogOutput(key = "Drive/Swerve/RealStates")
    private SwerveModuleState[] realStates;

    private final SwerveDriveOdometry kOdometry;

    private final SwerveDrivePoseEstimator kPoseEstimator;

    private final Field2d field = new Field2d();
    
    // An override value for the heading during teleop, used for auto heading, set to empty to default to normal heading control.
    private Optional<Rotation2d> teleopHeadingOverride = Optional.empty();

    // Instantiate controllers
    private final TeleopController kTeleopController = new TeleopController();
    private final HeadingController kHeadingController = new HeadingController(
        RobotConstants.DriveConstants().kHeadingController);
    private final HolonomicController kHolonomicController = new HolonomicController(
        RobotConstants.DriveConstants().kXTranslationalController, 
        RobotConstants.DriveConstants().kYTranslationalController);

    // Locks the azimuths in place prior to runnign the main commands
    // The only way to further simlpify this command is to use Java streams (Which are really weird).
    private Command lockAzimuthsSysIdCommand() {
        return new RunCommand(() -> {
            for(var module : kModules) {
                module.setAzimuthRotations(0.0d);
            }
        }, this).until(() -> {
            int modulesInPosition = 0;
            for (var input : kModuleInputs) {
                if(Math.abs(input.azimuthPositionRotations%0.5d) <= 0.05d) {
                    modulesInPosition++;
                }
            }
            return modulesInPosition >= 4;
        });
    }
    
    /******** COMMANDS ********/
    public Command resetGyroCommand() {
        return new InstantCommand(() -> resetGyro(), this);
    }

    public Command overrideTeleopHeadingCommand(Rotation2d angle) {
        return new InstantCommand(() -> overrideTeleopHeading(angle), this);
    }

    public Command releaseTeleopHeadingCommand() {
        return new InstantCommand(() -> releaseTeleopHeading(), this);
    }

    public Command resetAzimuthsCommand() {
        return new InstantCommand(() -> resetAzimuths(), this);
    }

    public Command setDriveStateCommand(driveState driveState) { return new InstantCommand(() -> setDriveState(driveState), this); }

    /******** COMMAND METHODS ********/
    public void resetGyro() {
        kGyro.resetGyro();
    }

    public void overrideTeleopHeading(Rotation2d angle) {
        teleopHeadingOverride = Optional.ofNullable(angle);
    }

    public void releaseTeleopHeading() {
        teleopHeadingOverride = Optional.empty();
    }

    public void resetAzimuths() {
        for(var module : kModules) {
            module.resetAzimuth();
        };
    }
    // Constructor for the drive subsystem
    public Drive(ModuleIO moduleFL, ModuleIO moduleFR, ModuleIO moduleBL, ModuleIO moduleBR, GyroIO gyro) {
        // What are we even doing guys...
        // These are literally constants... They don't change.
        System.out.println(RobotConstants.DriveConstants().kFLModuleOffsets.rotationalOffset().getRotations());
        System.out.println(RobotConstants.DriveConstants().kFRModuleOffsets.rotationalOffset().getRotations());
        System.out.println(RobotConstants.DriveConstants().kBLModuleOffsets.rotationalOffset().getRotations());
        System.out.println(RobotConstants.DriveConstants().kBRModuleOffsets.rotationalOffset().getRotations());

        kModules = new ModuleIO[] {
            moduleFL,
            moduleFR,
            moduleBL,
            moduleBR
        };
        kGyro = gyro;

        kKinematics = new SwerveDriveKinematics(
            RobotConstants.DriveConstants().kFLModuleOffsets.translationalOffset(),
            RobotConstants.DriveConstants().kFRModuleOffsets.translationalOffset(),
            RobotConstants.DriveConstants().kBLModuleOffsets.translationalOffset(),
            RobotConstants.DriveConstants().kBRModuleOffsets.translationalOffset()
        );

        kOdometry = new SwerveDriveOdometry(kKinematics, 
            kGyro.getGyroAngle().orElse(Rotation2d.kZero), 
            getModulePositions()
        );

        // Creating the pose estimator
        kPoseEstimator = new SwerveDrivePoseEstimator(kKinematics,
            kGyro.getGyroAngle().orElse(Rotation2d.kZero), 
            getModulePositions(), 
            kOdometry.getPoseMeters());

        SmartDashboard.putData("Field", field); // Field widget for dashboard

        kRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, // Default values
            (sysidState) -> Logger.recordOutput("Drive/SysIdState", sysidState.toString())),
            new SysIdRoutine.Mechanism((voltage) -> this.applySysIdVoltage(voltage.in(Volts)), 
            null, // AK will be logging the values here.
            this)
        );

        kHeadingController.supplyGyroAngle(() -> kGyro.getGyroAngle().orElse(Rotation2d.kZero));
    }

    public void supplyControllerInputs(DoubleSupplier xInputs, DoubleSupplier yInput, DoubleSupplier angleInput) {
        kTeleopController.supplyControllerInputs(xInputs, yInput, angleInput);
    }

    public void setDriveState(driveState driveState) { state = driveState; }

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
            lockAzimuthsSysIdCommand(),
            dynamicForward.andThen(new WaitCommand(1.0d)), dynamicReverse.andThen(new WaitCommand(1.0d)),
            quasistaticForward.andThen(new WaitCommand(1.0d)), quasistaticReverse.andThen(new WaitCommand(1.0d))
        );
    }

    // Used to direct set the voltages of each drive motor for PID.
    // NOTE: Before running a SysId test it is smart to ensure all Azimuth motors are facing relatively forwards.
    private void applySysIdVoltage(double driveVolts) {
        for(var module : kModules) {
            // Set drive goal to SysId given voltage.
            module.setDriveVoltage(driveVolts);
            // Set azimuth goal to zero degrees (To hold the azimuths in place).
            module.setAzimuthRotations(0.0d);
        }
    }

    /* Periodic Loop */
    @Override
    public void periodic() {
        // Update inputs for IO layers.
        for(int i = 0; i < kModuleInputs.length; ++i) {
            kModules[i].updateInputs(kModuleInputs[i]);
        }
        kGyro.updateInputs(kGyroInputs);

        // Update AK Logging.
        Logger.processInputs("Drive/ModuleFL", kModuleInputs[0]);
        Logger.processInputs("Drive/ModuleFR", kModuleInputs[1]);
        Logger.processInputs("Drive/ModuleBL", kModuleInputs[2]);
        Logger.processInputs("Drive/ModuleBR", kModuleInputs[3]);
        Logger.processInputs("Drive/Gyro", kGyroInputs);

        // Update Odometry and pose estimation.
        odometryPose = kOdometry.update(kGyro.getGyroAngle().orElse(Rotation2d.kZero), getModulePositions());
        updatePoseEstimation();
        field.setRobotPose(poseEstimator);

        // Internal State Handling.
        switch(state) {
            case TELEOP:
                // Compute chassis speeds.
                desiredSpeeds = kTeleopController.getDesiredSpeeds(false);
                stateUpdateTeleop();
                break;
            case TELEOP_SNIPER:
                desiredSpeeds = kTeleopController.getDesiredSpeeds(true);
                stateUpdateTeleop();
                break;
            case AUTON:
                desiredSpeeds = kHolonomicController.getVelocityGoal(getDesiredAutonSpeeds(), getDesiredAutonPose(), poseEstimator);
                desiredSpeeds.omegaRadiansPerSecond = kHeadingController.getOmega(Rotation2d.fromRadians(getDesiredAutonSpeeds().omegaRadiansPerSecond)).getRadians();
                releaseTeleopHeading();
                stateUpdateAutonomous();
                break;
            case SYSID:
                break;
            default:
                stateUpdateDisabled();
                break;
        }

        // Record real states constantly, even when teleop isn't running.
        realStates = new SwerveModuleState[] {
            new SwerveModuleState(rotationsToMeters(kModuleInputs[0].driveVelocityRPS), Rotation2d.fromRotations(kModuleInputs[0].azimuthPositionRotations)),
            new SwerveModuleState(rotationsToMeters(kModuleInputs[1].driveVelocityRPS), Rotation2d.fromRotations(kModuleInputs[1].azimuthPositionRotations)),
            new SwerveModuleState(rotationsToMeters(kModuleInputs[2].driveVelocityRPS), Rotation2d.fromRotations(kModuleInputs[2].azimuthPositionRotations)),
            new SwerveModuleState(rotationsToMeters(kModuleInputs[3].driveVelocityRPS), Rotation2d.fromRotations(kModuleInputs[3].azimuthPositionRotations))
        };

        // Update the gyro (usually for sim purposes)
        if(RobotConstants.Instance().kMode.equals(RobotConstants.mode.SIM)) {
            kGyro.updateGyro(Units.radiansToRotations(kKinematics.toChassisSpeeds(realStates).omegaRadiansPerSecond) * RobotConstants.Instance().kTimestep);
        }
    }

    /******** STATE UPDATES ********/
    private void stateUpdateTeleop() {
        // Discretized robot framed chassis speeds.
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, kGyro.getGyroAngle().orElse(Rotation2d.kZero));
        // Offset the desired angle in the heading controller, then compute the true omega value

        if(teleopHeadingOverride.isEmpty()) {
            kHeadingController.offsetHeading(Rotation2d.fromRadians(robotRelativeSpeeds.omegaRadiansPerSecond));
            robotRelativeSpeeds.omegaRadiansPerSecond = kHeadingController.getOmega().getRadians();
        } else {
            kHeadingController.setHeading(teleopHeadingOverride.get());
            robotRelativeSpeeds.omegaRadiansPerSecond = kHeadingController.getOmega().getRadians();
        }
        ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, RobotConstants.Instance().kTimestep);

        // Perform IK to get each indiviual module's goal setpoint and then desaturate to cap the speed.
        SwerveModuleState[] moduleStates = kKinematics.toSwerveModuleStates(discretizedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, RobotConstants.DriveConstants().kModuleSoftLimits.absoluteMaxDriveVelocityMPS());

        // Optimize the modules so they never rotate more than 90 degrees.
        states = moduleStates;

        optimizeModules();

        applyCosineOptimization();

        // Apply the modules goals to the actual motor.
        for(int i = 0; i < kModules.length; ++i) {
            kModules[i].setDriveRPS(metersToRotations(moduleStates[i].speedMetersPerSecond));
            kModules[i].setAzimuthRotations(moduleStates[i].angle.getRotations());
        }
    }

    private void stateUpdateAutonomous() {
        // Get the field relative speeds and convert them to robot relative speeds. Then convert them to swerve module states.
        ChassisSpeeds autonTargetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, kGyro.getGyroAngle().orElse(Rotation2d.kZero));
        kHeadingController.setHeading(getDesiredAutonPose().getRotation());
        SwerveModuleState[] autonModuleStates = kKinematics.toSwerveModuleStates(autonTargetSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(autonModuleStates, RobotConstants.DriveConstants().kModuleSoftLimits.absoluteMaxDriveVelocityMPS());
        states = autonModuleStates;

        optimizeModules();

        applyCosineOptimization();

        // Apply the modules goals to the actual motor.
        for(int i = 0; i < kModules.length; ++i) {
            kModules[i].setDriveRPS(metersToRotations(autonModuleStates[i].speedMetersPerSecond));
            kModules[i].setAzimuthRotations(autonModuleStates[i].angle.getRotations());
        }
    }

    private void stateUpdateDisabled() {
        for(int i = 0; i < kModules.length; ++i) {
            kModules[i].stopDrive();
            kModules[i].stopAzimuth();
        }
    }

    /******** HELPER METHODS ********/
    private void optimizeModules() {
        for(int i = 0; i < states.length; ++i) {
            states[i].optimize(Rotation2d.fromRotations(kModuleInputs[i].azimuthPositionRotations));
        }
    }

    private void applyCosineOptimization() {
        for(int i = 0; i < kModules.length; ++i) {
            double actualRotation = Units.rotationsToRadians(kModuleInputs[i].azimuthPositionRotations);

            // How aligned we are.
            double scalar = Math.cos(states[i].angle.getRadians() - actualRotation);

            // Apply scalar based on the alignment.
            states[i].speedMetersPerSecond *= scalar;
        }
    }

    // Updates the pose estimator based on vision and swerve odometry readings
    private void updatePoseEstimation() {
        // for(var visionEstimation : visionEstimations) {
        //     kPoseEstimator.addVisionMeasurement(visionEstimation.pose, visionEstimation.timestamp);
        // }
        poseEstimator = kPoseEstimator.update(kGyro.getGyroAngle().orElse(Rotation2d.kZero), getModulePositions());
    }

    private double rotationsToMeters(double rotations) {
        return Units.rotationsToRadians(rotations) * RobotConstants.DriveConstants().kModuleHardLimits.wheelRadiusMeters();
    }
    
    private double metersToRotations(double meters) {
        return meters / ((2 * Math.PI) * RobotConstants.DriveConstants().kModuleHardLimits.wheelRadiusMeters());
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(rotationsToMeters(kModuleInputs[0].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[0].azimuthPositionRotations))),
            new SwerveModulePosition(rotationsToMeters(kModuleInputs[1].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[1].azimuthPositionRotations))),
            new SwerveModulePosition(rotationsToMeters(kModuleInputs[2].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[2].azimuthPositionRotations))),
            new SwerveModulePosition(rotationsToMeters(kModuleInputs[3].drivePositionRotations), new Rotation2d(Units.rotationsToRadians(kModuleInputs[3].azimuthPositionRotations)))
        };
    }
    
    public driveState getDriveState() { return state; }

    // TODO implement getDesiredAutonPose().
    private Pose2d getDesiredAutonPose() { return new Pose2d(); }

    // TODO implement getDesiredAutonSpeeds().
    private ChassisSpeeds getDesiredAutonSpeeds() { return new ChassisSpeeds(); }

}
