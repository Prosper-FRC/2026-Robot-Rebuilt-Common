// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Factories.DriveFactory;
import frc.robot.Factories.IndexerFactory;
import frc.robot.Factories.IntakeFactory;
import frc.robot.Factories.ShooterFactory;
import frc.robot.Factories.SubsystemFactory.SubsystemType;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.Drive.driveState;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Superstructure.Superstructure;
import frc.robot.Superstructure.Superstructure.robotState;

// This whole file is currently really scuffed, I intend to fix it later.
public class RobotContainer {
    // Declare robot constants and subsystems.
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.Instance().kDriveControllerPort);
    public final CommandXboxController kOperatorController = new CommandXboxController(RobotConstants.Instance().kOperatorControllerPort);
    public final Drive kDrive;
    public final Intake kIntake;
    public final Indexer kIndexer;
    public final Shooter kShooter;
    public final Superstructure kSuperstructure;


    public RobotContainer() {
        kDrive = DriveFactory.create(SubsystemType.SIM);
        kIntake = IntakeFactory.create(SubsystemType.SIM);
        kIndexer = IndexerFactory.create(SubsystemType.SIM);
        kShooter = ShooterFactory.create(SubsystemType.SIM);

        kSuperstructure = new Superstructure(
            kDrive, 
            kIntake, 
            kIndexer, 
            kShooter
        );

        configureBindings();
    }

    // Bind buttons to hardware.
    private void configureBindings() {
        kDrive.setDefaultCommand(kDrive.setDriveStateCommand(driveState.TELEOP));

        kDrive.supplyControllerInputs(() -> kDriveController.getLeftX(), () -> kDriveController.getLeftY(), () -> kDriveController.getRightX());

        kDriveController.y()
            .onTrue(kDrive.resetGyroCommand());

        kDriveController.x()
            .whileTrue(kDrive.setDriveStateCommandContinuous(driveState.TELEOP_SNIPER));
        
        kDriveController.a()
            .whileTrue(kDrive.overrideTeleopHeadingCommand(Rotation2d.fromRotations(0.0d)))
            .onFalse(kDrive.releaseTeleopHeadingCommand());

        kOperatorController.y()
            .onTrue(kSuperstructure.setRobotStateCommand(robotState.DEFAULT_CONFIGURATION).alongWith(kIntake.setRollerVoltageCommand(0.0d)));
        
        kOperatorController.leftTrigger()
            .onTrue(kIntake.setRollerVoltageCommand(-4.0d).alongWith(kSuperstructure.setRobotStateCommand(robotState.ACTIVE_CONFIGURATION)));
        kOperatorController.leftBumper()
            .onTrue(kIntake.setRollerVoltageCommand(4.0d).alongWith(kSuperstructure.setRobotStateCommand(robotState.ACTIVE_CONFIGURATION)));

    }
}