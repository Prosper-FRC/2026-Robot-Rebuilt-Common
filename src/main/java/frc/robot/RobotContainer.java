// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Factories.DriveFactory;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.Drive.DriveState;

public class RobotContainer {
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.Instance().kDriveControllerPort);
    public final CommandXboxController kOperatorController = new CommandXboxController(RobotConstants.Instance().kOperatorControllerPort);

    public final boolean kUseCompetitionBindings = false;
    
    public final Drive kDrive;

    public RobotContainer() {
        kDrive = DriveFactory.createSim();
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        if(kUseCompetitionBindings) {
            // TODO: add these.
        } else {
            kDrive.supplyControllerInputs(
                () -> kDriveController.getLeftX(), 
                () -> kDriveController.getLeftY(), 
                () -> kDriveController.getRightX()
            );

            kDriveController.y()
                .onTrue(kDrive.resetGyroCommand());

            kDriveController.a()
                .onTrue(kDrive.setDriveStateCommand(DriveState.AUTON)
                    .andThen(kDrive.followTrajectoryCommand(new Pose2d())))
                .onFalse(kDrive.setDriveStateCommand(DriveState.TELEOP));

            kDriveController.b()
                .onTrue(kDrive.setDriveStateCommand(DriveState.SYSID).andThen(kDrive.getSysIdCommand()))
                .onFalse(kDrive.setDriveStateCommand(DriveState.TELEOP));
        }
    }
}