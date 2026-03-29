package frc.robot.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.intakeState;
import frc.robot.Subsystems.Shooter.Shooter;
public class Superstructure extends SubsystemBase {
    public static enum robotState {
        DEFAULT_CONFIGURATION, // Intake up, rollers stopped, hoppers and shooters stopped.
        ACTIVE_CONFIGURATION, // Intake out, rollers active, hoppers slow, and shooter stopped.
        ACTIVE_SCORE_CONFIGURATION // Intake out, rollers active, hoppers fast, and shooter enabled.
    }
    public robotState state = robotState.DEFAULT_CONFIGURATION;

    public final Drive kDrive;
    public final Intake kIntake;
    public final Indexer kIndexer;
    public final Shooter kShooter;

    public final Trigger kDefaultConfigurationTrigger = new Trigger(() -> state == robotState.DEFAULT_CONFIGURATION);
    public final Trigger kActiveConfigurationTrigger = new Trigger(() -> state == robotState.ACTIVE_CONFIGURATION);
    public final Trigger kActiveScoreConfigurationTrigger = new Trigger(() -> state == robotState.ACTIVE_SCORE_CONFIGURATION);

    public final Trigger kIsPivotAtGoal;

    public Superstructure(Drive drive, Intake intake, Indexer indexer, Shooter shooter) {
        kDrive = drive;
        kIntake = intake;
        kIndexer = indexer;
        kShooter = shooter;

        kIsPivotAtGoal = new Trigger(() -> kIntake.isAtGoal());

        kDefaultConfigurationTrigger
            .onTrue(kIntake.setIntakeStateCommand(intakeState.Stowed)
            .alongWith(kIndexer.stopHoppersAndBallTunnelCommand()
            .alongWith(kShooter.stopShooterCommand())));
        kActiveConfigurationTrigger
            .onTrue(kIntake.setIntakeStateCommand(intakeState.Deployed).alongWith(kShooter.stopShooterCommand()));
        kActiveConfigurationTrigger.and(kIsPivotAtGoal)
            .whileTrue(kIndexer.setHoppersAndBallTunnelCommand(3.0d, 3.0d));
        kActiveScoreConfigurationTrigger
            .onTrue(kIntake.setIntakeStateCommand(intakeState.Deployed)
            .alongWith(kShooter.setShooterCommand(45.0d, 0.0d)));
        kActiveScoreConfigurationTrigger.and(kIsPivotAtGoal)
            .whileTrue((kIndexer.setHoppersAndBallTunnelCommand(9.0d, 10.0d)));
    }

    public void setRobotState(robotState state) {
        this.state = state;
    }
    public Command setRobotStateCommand(robotState state) {
        return new InstantCommand(() -> setRobotState(state));
    }
}
