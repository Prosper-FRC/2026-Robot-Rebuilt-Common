package frc.robot.Superstructure;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroPigeon2;
import frc.robot.Subsystems.Drive.GyroSim;
import frc.robot.Subsystems.Drive.ModuleSim;
import frc.robot.Subsystems.Drive.ModuleTalonFX;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants;
import frc.robot.Subsystems.Indexer.HopperTalonFX;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerConstants.IndexerConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.intakeState;
import frc.robot.Subsystems.Intake.PivotSim;
import frc.robot.Subsystems.Intake.PivotTalonFX;
import frc.robot.Subsystems.Intake.RollerSim;
import frc.robot.Subsystems.Intake.RollerTalonFX;
import frc.robot.Subsystems.Shooter.FlywheelTalonFX;
import frc.robot.Subsystems.Shooter.HoodRev;
import frc.robot.Subsystems.Shooter.Shooter;

public class Superstructure {
    public static enum robotState {
        DEFAULT_CONFIGURATION,
        ACTIVE_CONFIGURATION,
        ACTIVE_SCORE_CONFIGURATION
    }
    public robotState state = robotState.DEFAULT_CONFIGURATION;

    private final int kTeamNumber = RobotConstants.Instance().kTeamNumber;

    public final Drive kDrive;
    public final Intake kIntake;
    public final Indexer kIndexer;
    public final Shooter kShooter;

    public final TalonFX kBallTunnelMotor;
    public final TalonFX kHopper1;
    public final TalonFX kHopper2;
    public final TalonFX kHopper3;


    private final DriveConstants kDConsts = RobotConstants.DriveConstants();
    private final IndexerConstants kIConsts = RobotConstants.IndexerConstants();

    public Superstructure(boolean useDrive, boolean useIntake, boolean useIndexer, boolean useShooter) {
        kBallTunnelMotor = new TalonFX(44);
        kHopper1 = new TalonFX(41);
        kHopper2 = new TalonFX(42);
        kHopper3 = new TalonFX(43);
        if(kTeamNumber == 0) {
            // Sim init
            kDrive = new Drive(new ModuleSim(),
            new ModuleSim(), 
            new ModuleSim(), 
            new ModuleSim(), 
            new GyroSim());
            
            kIntake = new Intake(new RollerSim(), new PivotSim());

            kIndexer = Indexer.NoOp;

            kShooter = Shooter.NoOp;

        } else {
            if(useDrive) {
                kDrive = new Drive(
                    new ModuleTalonFX(kDConsts.kFLModuleIDs, kDConsts.kFLModuleOffsets, kDConsts.kModuleGains, kDConsts.kCANBusInstance), 
                    new ModuleTalonFX(kDConsts.kFRModuleIDs, kDConsts.kFRModuleOffsets, kDConsts.kModuleGains, kDConsts.kCANBusInstance), 
                    new ModuleTalonFX(kDConsts.kBLModuleIDs, kDConsts.kBLModuleOffsets, kDConsts.kModuleGains, kDConsts.kCANBusInstance), 
                    new ModuleTalonFX(kDConsts.kBRModuleIDs, kDConsts.kBRModuleOffsets, kDConsts.kModuleGains, kDConsts.kCANBusInstance), 
                    new GyroPigeon2(kDConsts.kGyroID, kDConsts.kGyroOffsets, kDConsts.kCANBusInstance));
            } else {
                kDrive = Drive.NoOp;
            }
            
            if(useIntake) {
                kIntake = new Intake(
                    new RollerTalonFX(),
                    new PivotTalonFX()
                );
            } else {
                kIntake = Intake.NoOp;
            }

            if(useIndexer) {
                kIndexer = new Indexer(
                    new HopperTalonFX(kIConsts.kHopper1Id), 
                    new HopperTalonFX(kIConsts.kHopper2Id), 
                    new HopperTalonFX(kIConsts.kHopper3Id));
            } else {
                kIndexer = Indexer.NoOp;
            }
            
            if(useShooter) {
                kShooter = new Shooter(new FlywheelTalonFX(), new HoodRev());
            } else {
                kShooter = Shooter.NoOp;
            }
        }
    }

    // Binds a drive command to the given command xb controller.
    public void bindDriveCommands(CommandXboxController toBind) {
            kDrive.setDefaultCommand(kDrive.setDriveStateCommand(Drive.driveState.DISABLED));
            DriverStation.silenceJoystickConnectionWarning(true);

            kDrive.supplyControllerInputs(
                () -> toBind.getLeftX(), 
                () -> toBind.getLeftY(), 
                () -> toBind.getRightX());

            toBind.b().debounce(0.25, DebounceType.kRising)
                .onTrue(kDrive.overrideTeleopHeadingCommand(Rotation2d.kZero))
                .onFalse(kDrive.releaseTeleopHeadingCommand());
        
            toBind.y().debounce(0.1d, DebounceType.kRising)
                .onTrue(kDrive.resetGyroCommand());
    }

    public void bindIntakeCommands(CommandXboxController toBind) {
        toBind.rightBumper().whileTrue(kIntake.setRollerVoltageCommand(4.0d))
            .onFalse(kIntake.setRollerVoltageCommand(0.0d));
        toBind.leftBumper().whileTrue(kIntake.setRollerVoltageCommand(-4.0d))
            .onFalse(kIntake.setRollerVoltageCommand(0.0d));
        toBind.leftTrigger().onTrue(kIntake.setIntakeStateCommand(intakeState.Deployed)); // pivot button bindings, don't go through the state layer if this doesn't work in the morning
            //.onFalse(kIntake.setIntakeStateCommand(intakeState.Stowed));
        toBind.rightTrigger().onTrue(kIntake.setIntakeStateCommand(intakeState.Stowed)); 
            //.onFalse(kIntake.setIntakeStateCommand(intakeState.Stowed));
    }

    public void bindIndexerCommands(CommandXboxController toBind) {
        toBind.a()
            .whileTrue(new InstantCommand(() -> kBallTunnelMotor.setControl(new VoltageOut(5.0d))))
            .onFalse(new InstantCommand(() -> kBallTunnelMotor.setControl(new NeutralOut())));
        toBind.a()
            .whileTrue(new InstantCommand(() -> kHopper1.setControl(new VoltageOut(8.0d))))
            .onFalse(new InstantCommand(() -> kHopper1.setControl(new NeutralOut())));
        toBind.a()
            .whileTrue(new InstantCommand(() -> kHopper2.setControl(new VoltageOut(-10.0d))))
            .onFalse(new InstantCommand(() -> kHopper2.setControl(new NeutralOut())));
        toBind.a()
            .whileTrue(new InstantCommand(() -> kHopper3.setControl(new VoltageOut(-10.0d))))
            .onFalse(new InstantCommand(() -> kHopper3.setControl(new NeutralOut())));
    } 

    public void bindShooterCommands(CommandXboxController toBind) {
        toBind.rightTrigger(0.5d)
            .whileTrue(kShooter.setShooterCommand(45.0d, 0.0d))
            .onFalse(kShooter.stopShooterCommand());
    }
}
