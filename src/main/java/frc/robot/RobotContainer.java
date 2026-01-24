// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Drive.GyroIO;
import frc.robot.Subsystems.Drive.GyroPigeon2;
import frc.robot.Subsystems.Drive.GyroSim;
import frc.robot.Subsystems.Drive.ModuleIO;
import frc.robot.Subsystems.Drive.ModuleSim;
import frc.robot.Subsystems.Drive.ModuleTalonFX;

public class RobotContainer {
    public final Drive kDrive;
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.getInstance().kDriveControllerPort);

    public RobotContainer() {
        switch (RobotConstants.getInstance().kMode) {
            case REAL:
                kDrive = new Drive(
                    new ModuleTalonFX[] {
                        new ModuleTalonFX(DriveConstants.getInstance().kFLModuleIDs),
                        new ModuleTalonFX(DriveConstants.getInstance().kFRModuleIDs),
                        new ModuleTalonFX(DriveConstants.getInstance().kBLModuleIDs),
                        new ModuleTalonFX(DriveConstants.getInstance().kBRModuleIDs)
                    }, new GyroPigeon2());
                break;
            case REPLAY:
                kDrive = new Drive(new ModuleIO[] {}, new GyroIO() {});
                break;
            case SIM:
                kDrive = new Drive(new ModuleSim[] {
                    new ModuleSim(),
                    new ModuleSim(),
                    new ModuleSim(),
                    new ModuleSim()
                }, new GyroSim());
                break;
            default:
                kDrive = new Drive(new ModuleIO[] {}, new GyroIO() {});
                break;
        }

        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);;

        kDrive.assignJoysticks(() -> kDriveController.getLeftX(), () -> kDriveController.getLeftY(), () -> kDriveController.getRightX());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
