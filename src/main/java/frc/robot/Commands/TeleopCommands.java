package frc.robot.Commands;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.Indexer.indexerState;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.intakePivotState;
import frc.robot.Subsystems.Intake.Intake.intakeRollerState;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.shooterState;

public class TeleopCommands {
    private final Drive kDrive;
    private final Intake kIntake;
    private final Indexer kIndexer;
    private final Shooter kShooter;

    @AutoLogOutput(key = "Intake/IsAtGoal")
    private Trigger isIntakeAtGoal;

    @AutoLogOutput(key = "Shooter/Flywheel/IsAtGoal")
    private Trigger isShooterAtGoal;

    @AutoLogOutput(key = "Intake/IsDeployed")
    private Trigger isIntakeDeployed;

    public TeleopCommands(Drive drive, Intake intake, Indexer indexer, Shooter shooter) {
        kDrive = drive;
        kIntake = intake;
        kIndexer = indexer;
        kShooter = shooter;

        isIntakeAtGoal = new Trigger(() -> kIntake.isAtGoal());
        isShooterAtGoal = new Trigger(() -> kShooter.isShooterSpunUp());
        isIntakeDeployed = new Trigger(() -> {
            return kIntake.pivotState.equals(intakePivotState.Deployed);
        });
    }
    
    public Command superstructureStateDefault() {
        return kIntake.setIntakeStateCommand(intakePivotState.Stowed, intakeRollerState.Inactive)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Inactive))
                .alongWith(kShooter.setShooterStateCommand(shooterState.Inactive));
    }

    public Command shiftToSuperstructureStateIntake() {
        return kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Inactive);
    }

    public Command superstructureStateIntake() {
        return new WaitUntilCommand(isIntakeAtGoal)
                .andThen(kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Intake)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Active)));
    }

    public Command exitSuperstructureStateIntake() {
        return kIndexer.setIndexerStateCommand(indexerState.Inactive)
            .alongWith(kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Idling));
    }

    public Command shiftToSuperstructureStateShooter() {
        return kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Inactive)
            .alongWith(kShooter.setShooterStateCommand(shooterState.Active));
    }

    public Command superstructureStateShooter() {
        return new WaitUntilCommand(isIntakeAtGoal).andThen(
            kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Idling)
            .alongWith(new WaitUntilCommand(isShooterAtGoal)
            .andThen(kIndexer.setIndexerStateCommand(indexerState.Active))));
    }

    public Command exitSuperstructureStateShooter() {
        return kShooter.setShooterStateCommand(shooterState.Inactive)
            .alongWith(kIndexer.setIndexerStateCommand(indexerState.Inactive));
    }

    public Command shiftToSuperstructureStateOuttake() {
        return kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Inactive).unless(isIntakeAtGoal.and(isIntakeDeployed));
    }

    public Command superstructureStateOuttake() {
        return new WaitUntilCommand(isIntakeAtGoal)
            .andThen(kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Outtake)
            .alongWith(kIndexer.setIndexerStateCommand(indexerState.Active)));
    }

    public Command exitSuperstructureStateOuttake() {
        return kIndexer.setIndexerStateCommand(indexerState.Inactive)
            .alongWith(kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Inactive));
    }
}
