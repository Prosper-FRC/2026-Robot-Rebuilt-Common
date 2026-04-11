// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutonCommands;
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
import frc.robot.Subsystems.Intake.Intake.intakeRollerState;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.shooterState;

public class RobotContainer {
    public static enum robotState {
        UNLOCKED,
        DEFAULT_CONFIGURATION,
        INTAKE_CONFIGURATION,
        OUTTAKE_CONFIGURATION,
        SHOOTER_CONFIGURATION
    }
    public robotState rState = robotState.DEFAULT_CONFIGURATION;

    // Declare robot constants and subsystems.
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.Instance().kDriveControllerPort);
    public final CommandXboxController kOperatorController = new CommandXboxController(RobotConstants.Instance().kOperatorControllerPort);
    public final Drive kDrive;
    public final Intake kIntake;
    public final Indexer kIndexer;
    public final Shooter kShooter;

    public final AutonCommands kACommands;

    public Command setRobotState(robotState state) {
        return new InstantCommand(() -> rState = state);
    }


    public RobotContainer() {
        // We're manually changing these for now.
        kDrive = DriveFactory.create(SubsystemType.REAL);
        kIntake = IntakeFactory.create(SubsystemType.REAL);
        kIndexer = IndexerFactory.create(SubsystemType.REAL);
        kShooter = ShooterFactory.create(SubsystemType.REAL);
    
        kACommands = new AutonCommands(kDrive, kShooter, kIndexer);
        
        configureBindings();
    }

    // Bind buttons to hardware.
    private void configureBindings() {

        kDrive.setDefaultCommand(kDrive.setDriveStateCommand(driveState.TELEOP));

        kDrive.supplyControllerInputs(() -> kDriveController.getLeftX(), () -> kDriveController.getLeftY(), () -> -kDriveController.getRightX());

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
            .onTrue(kIntake.setIntakeRollerStateCommand(intakeRollerState.Outtake))
            .onFalse(kIntake.setIntakeRollerStateCommand(intakeRollerState.Idling));

        kOperatorController.leftBumper().and(kOperatorController.leftTrigger().negate())
            .onTrue(kIntake.setIntakeRollerStateCommand(intakeRollerState.Intake))
            .onFalse(kIntake.setIntakeRollerStateCommand(intakeRollerState.Inactive));

        kOperatorController.y()
            .onTrue(kIntake.toggleIntakePivotState());

        kShooter.supplyHoodAxis(() -> kOperatorController.getLeftY());
    }
}