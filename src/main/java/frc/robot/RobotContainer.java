// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroSim;
import frc.robot.Subsystems.Drive.ModuleSim;

public class RobotContainer {
    // Declare robot constants and subsystems.
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.Instance().kDriveControllerPort);
    public Drive kDrive;

    public RobotContainer() {
        // Initializes the subsystems.
        // TODO Replace this process with a superstructure?
        switch (RobotConstants.Instance().kMode) {
            case REAL:
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

        kDrive.supplyControllerInputs(() -> kDriveController.getLeftX(), () -> kDriveController.getLeftY(), () -> kDriveController.getRightX());
    }
}
