package frc.robot.Superstructure;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroPigeon2;
import frc.robot.Subsystems.Drive.GyroSim;
import frc.robot.Subsystems.Drive.ModuleSim;
import frc.robot.Subsystems.Drive.ModuleTalonFX;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.PivotSim;
import frc.robot.Subsystems.Intake.PivotTalonFX;
import frc.robot.Subsystems.Intake.RollerSim;
import frc.robot.Subsystems.Intake.RollerTalonFX;
import frc.robot.Subsystems.Intake.Intake.intakeState;

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

    private final DriveConstants kDConsts = RobotConstants.DriveConstants();

    public Superstructure(boolean useDrive, boolean useIntake, boolean useIndexer, boolean useShooter) {
        if(kTeamNumber == 0) {
            // Sim init
            kDrive = new Drive(new ModuleSim(),
            new ModuleSim(), 
            new ModuleSim(), 
            new ModuleSim(), 
            new GyroSim());
            
            kIntake = new Intake(new RollerSim(), new PivotSim());
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
        
            toBind.a().debounce(0.25d, DebounceType.kRising)
                .onTrue(new InstantCommand(() -> kDrive.setDriveState(Drive.driveState.SYSID)).andThen(kDrive.getSysIdCommand()))
                .onFalse(kDrive.getDefaultCommand());

            toBind.y().debounce(0.1d, DebounceType.kRising)
                .onTrue(kDrive.resetGyroCommand());
    }

    public void bindIntakeCommands(CommandXboxController toBind) {
        kIntake.setDefaultCommand(kIntake.setIntakeStateCommand(intakeState.Stowed));

        toBind.a()
            .whileTrue(kIntake.setIntakeStateCommand(intakeState.Deployed))
            .onFalse(kIntake.getDefaultCommand());
    }
}
