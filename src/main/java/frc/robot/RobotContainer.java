// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.estimation.TargetModel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Factories.DriveFactory;
import frc.robot.Factories.IndexerFactory;
import frc.robot.Factories.IntakeFactory;
import frc.robot.Factories.ShooterFactory;
import frc.robot.Factories.SubsystemFactory.SubsystemType;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.Drive.driveState;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.Indexer.indexerState;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.intakePivotState;
import frc.robot.Subsystems.Intake.Intake.intakeRollerState;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.shooterState;

// This whole file is currently really scuffed, I intend to fix it later.
public class RobotContainer {
    // Declare robot constants and subsystems.
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.Instance().kDriveControllerPort);
    public final CommandXboxController kOperatorController = new CommandXboxController(RobotConstants.Instance().kOperatorControllerPort);
    public final Drive kDrive;
    public final Intake kIntake;
    public final Indexer kIndexer;
    public final Shooter kShooter;

    public RobotContainer() {
        // We're manually changing these for now.
        kDrive = DriveFactory.create(SubsystemType.SIM);
        kIntake = IntakeFactory.create(SubsystemType.SIM);
        kIndexer = IndexerFactory.create(SubsystemType.SIM);
        kShooter = ShooterFactory.create(SubsystemType.SIM);

        configureBindings();
    }

    @AutoLogOutput(key = "Intake/IsAtGoal")
    private Trigger isIntakeAtGoal;

    @AutoLogOutput(key = "Shooter/Flywheel/IsAtGoal")
    private Trigger isShooterAtGoal;

    @AutoLogOutput(key = "Intake/IsDeployed")
    private Trigger isIntakeDeployed;

    // Bind buttons to hardware.
    private void configureBindings() {
        isIntakeAtGoal = new Trigger(() -> kIntake.isAtGoal());
        isShooterAtGoal = new Trigger(() -> kShooter.isShooterSpunUp());
        isIntakeDeployed = new Trigger(() -> {
            return kIntake.pivotState.equals(intakePivotState.Deployed);
        });

        kDrive.setDefaultCommand(kDrive.setDriveStateCommand(driveState.TELEOP));

        kDrive.supplyControllerInputs(() -> kDriveController.getLeftX(), () -> kDriveController.getLeftY(), () -> kDriveController.getRightX());

        kDriveController.y()
            .onTrue(kDrive.resetGyroCommand()
        );

        kDriveController.x()
            .whileTrue(kDrive.setDriveStateCommandContinuous(driveState.TELEOP_SNIPER)
        );
        
        kDriveController.a()
            .whileTrue(kDrive.overrideTeleopHeadingCommand(Rotation2d.fromRotations(0.0d)))
            .onFalse(kDrive.releaseTeleopHeadingCommand()
        );

        // Note: We can only shift commands when the intake is at its most recent desired goal.
        kOperatorController.rightTrigger(0.75d)
            .onTrue(
                kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Inactive)
                .alongWith(kShooter.setShooterStateCommand(shooterState.Active)))
            .whileTrue (
                new WaitUntilCommand(isIntakeAtGoal).andThen(
                    kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Idling)
                    .alongWith(new WaitUntilCommand(isShooterAtGoal)
                        .andThen(kIndexer.setIndexerStateCommand(indexerState.Active)))
                )
            )
            .onFalse(
                kShooter.setShooterStateCommand(shooterState.Inactive)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Inactive))
            );

        kOperatorController.rightBumper()
            .onTrue(kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Inactive))
            .whileTrue(new WaitUntilCommand(isIntakeAtGoal)
                .andThen(kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Intake)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Active))))
            .onFalse(kIndexer.setIndexerStateCommand(indexerState.Inactive)
            .alongWith(kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Idling)));

        kOperatorController.leftBumper()
            .onTrue(kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Inactive).unless(isIntakeAtGoal.and(isIntakeDeployed)))
            .whileTrue(new WaitUntilCommand(isIntakeAtGoal)
                .andThen(kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Outtake)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Active))))
            .onFalse(kIndexer.setIndexerStateCommand(indexerState.Inactive)
            .alongWith(kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Inactive)));

        kOperatorController.y()
            .onTrue(
                kIntake.setIntakeStateCommand(intakePivotState.Stowed, intakeRollerState.Inactive)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Inactive))
                .alongWith(kShooter.setShooterStateCommand(shooterState.Inactive))
            );
    }
}