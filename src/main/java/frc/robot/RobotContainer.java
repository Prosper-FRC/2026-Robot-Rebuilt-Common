// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.*;
import frc.robot.Superstructure.Superstructure;

// This whole file is currently really scuffed, I intend to fix it later.
public class RobotContainer {
    // Declare robot constants and subsystems.
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.Instance().kDriveControllerPort);
    public final Superstructure kSuperstructure = new Superstructure(true, false, false, false);

    public RobotContainer() {
        configureBindings();
    }

    // Bind buttons to hardware.
    private void configureBindings() {
        kSuperstructure.kDrive.setDefaultCommand(kSuperstructure.kDrive.setDriveStateCommand(Drive.driveState.DISABLED));
        DriverStation.silenceJoystickConnectionWarning(true);

        // I'm not even going to ask why this was pushed to remote.
        // if (RobotController.getTeamNumber() == 9105) {
        //     System.out.println("9105");
        //     kDrive.supplyControllerInputs(() -> -kDriveController.getLeftX(), () -> -kDriveController.getLeftY(), () -> kDriveController.getRightX());
        // } else {
        //     System.out.println("5411, 9492");
        //     kDrive.supplyControllerInputs(() -> kDriveController.getLeftX(), () -> kDriveController.getLeftY(), () -> kDriveController.getRightX());
        // }

        kSuperstructure.kDrive.supplyControllerInputs(() -> kDriveController.getLeftX(), () -> kDriveController.getLeftY(), () -> kDriveController.getRightX());

        kDriveController.b().debounce(0.25, DebounceType.kRising)
            .onTrue(kSuperstructure.kDrive.overrideTeleopHeadingCommand(Rotation2d.kZero))
            .onFalse(kSuperstructure.kDrive.releaseTeleopHeadingCommand());
    
        kDriveController.a().debounce(0.25d, DebounceType.kRising)
            .onTrue(new InstantCommand(() -> kSuperstructure.kDrive.setDriveState(Drive.driveState.SYSID)).andThen(kSuperstructure.kDrive.getSysIdCommand()))
            .onFalse(kSuperstructure.kDrive.getDefaultCommand());

        kDriveController.y().debounce(0.1d, DebounceType.kRising)
            .onTrue(kSuperstructure.kDrive.resetGyroCommand());
    }
}
