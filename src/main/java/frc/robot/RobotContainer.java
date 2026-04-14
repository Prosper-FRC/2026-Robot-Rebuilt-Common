// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Factories.DriveFactory;
import frc.robot.Factories.IndexerFactory;
import frc.robot.Factories.IntakeFactory;
import frc.robot.Factories.ShooterFactory;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.Drive.DriveState;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.Indexer.indexerState;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.intakeRollerState;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.shooterState;

public class RobotContainer {
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.Instance().kDriveControllerPort);
    public final CommandXboxController kOperatorController = new CommandXboxController(RobotConstants.Instance().kOperatorControllerPort);

    public final boolean kUseCompetitionBindings = false;
    
    public final Drive kDrive;
    public final Indexer kIndexer;
    public final Intake kIntake;
    public final Shooter kShooter;

    public RobotContainer() {
        kDrive = DriveFactory.createSim();
        kIndexer = IndexerFactory.createSim();
        kIntake = IntakeFactory.createSim();
        kShooter = ShooterFactory.createSim();

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        kDrive.setDefaultCommand(kDrive.setDriveStateCommand(DriveState.TELEOP));

        kDrive.supplyControllerInputs(() -> kDriveController.getLeftX(), () -> kDriveController.getLeftY(), () -> -kDriveController.getRightX());

        kDriveController.y()
            .onTrue(kDrive.resetGyroCommand()
        );

        // Note: We can only shift commands when the intake is at its most recent desired goal.
        kOperatorController.rightBumper().and(kOperatorController.rightTrigger().negate())
            .onTrue(kIndexer.setIndexerStateCommand(indexerState.Active_Hoppers))
            .onFalse(kIndexer.setIndexerStateCommand(indexerState.Inactive));

        kOperatorController.rightTrigger().and(kOperatorController.rightBumper().negate())
            .onTrue(kShooter.setShooterStateCommand(shooterState.Active)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Active_Ball_Tunnel)))
            .onFalse(kShooter.setShooterStateCommand(shooterState.Inactive)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Inactive)));

        kOperatorController.rightBumper().and(kOperatorController.rightTrigger())
            .onTrue(kShooter.setShooterStateCommand(shooterState.Active)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Active)))
            .onFalse(kShooter.setShooterStateCommand(shooterState.Inactive)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Inactive)));

        kOperatorController.leftTrigger().and(kOperatorController.leftBumper().negate())
            .onTrue(kIntake.setIntakeRollerStateCommand(intakeRollerState.Outtake)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Reverse)))
            .onFalse(kIntake.setIntakeRollerStateCommand(intakeRollerState.Inactive)
                .alongWith(kIndexer.setIndexerStateCommand(indexerState.Inactive)));

        kOperatorController.leftBumper().and(kOperatorController.leftTrigger().negate())
            .onTrue(kIntake.setIntakeRollerStateCommand(intakeRollerState.Intake))
            .onFalse(kIntake.setIntakeRollerStateCommand(intakeRollerState.Idling));

        kOperatorController.y()
            .onTrue(kIntake.toggleIntakePivotState());
    }
}