// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.*;

// This whole file is currently really scuffed, I intend to fix it later.
public class RobotContainer {
    // Declare robot constants and subsystems.
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.Instance().kDriveControllerPort);
    public Drive kDrive;

    public RobotContainer() {
        // Initializes the subsystems.
        // TODO Replace this process with a superstructure?
        switch (RobotConstants.Instance().kMode) {
            case REAL:
                kDrive = new Drive(
                    new ModuleTalonFX(RobotConstants.DriveConstants().kFLModuleIDs, RobotConstants.DriveConstants().kFLModuleOffsets, RobotConstants.DriveConstants().kModuleGains, RobotConstants.DriveConstants().kCANBusInstance),
                    new ModuleTalonFX(RobotConstants.DriveConstants().kFRModuleIDs, RobotConstants.DriveConstants().kFRModuleOffsets, RobotConstants.DriveConstants().kModuleGains, RobotConstants.DriveConstants().kCANBusInstance),
                    new ModuleTalonFX(RobotConstants.DriveConstants().kBLModuleIDs, RobotConstants.DriveConstants().kBLModuleOffsets, RobotConstants.DriveConstants().kModuleGains, RobotConstants.DriveConstants().kCANBusInstance),
                    new ModuleTalonFX(RobotConstants.DriveConstants().kBRModuleIDs, RobotConstants.DriveConstants().kBRModuleOffsets, RobotConstants.DriveConstants().kModuleGains, RobotConstants.DriveConstants().kCANBusInstance),
                    new GyroPigeon2(RobotConstants.DriveConstants().kGyroID, RobotConstants.DriveConstants().kGyroOffsets, RobotConstants.DriveConstants().kCANBusInstance)
                );
                break;
            case REPLAY:
                break;
            case SIM:
                kDrive = new Drive(
                    new ModuleSim(), 
                    new ModuleSim(), 
                    new ModuleSim(), 
                    new ModuleSim(), 
                    new GyroSim()
                );
                break;
            default:
                break;
        }

        configureBindings();
    }

    // Bind buttons to hardware.
    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        kDrive.setDefaultCommand(new InstantCommand(() -> kDrive.setDriveState(Drive.driveState.TELEOP), kDrive));

        kDrive.supplyControllerInputs(() -> -kDriveController.getLeftX(), () -> -kDriveController.getLeftY(), () -> -kDriveController.getRightX());
    
        kDriveController.a().debounce(0.25d, DebounceType.kRising)
            .onTrue(new InstantCommand(() -> kDrive.setDriveState(Drive.driveState.SYSID)).andThen(kDrive.getSysIdCommand()))
            .onFalse(kDrive.getDefaultCommand());

        kDriveController.y().debounce(0.1d, DebounceType.kRising)
            .onTrue(kDrive.resetGyro());
    }
}
