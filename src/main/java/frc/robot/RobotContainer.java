// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Superstructure.Superstructure;

// This whole file is currently really scuffed, I intend to fix it later.
public class RobotContainer {
    // Declare robot constants and subsystems.
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.Instance().kDriveControllerPort);
    public final CommandXboxController kOperatorController = new CommandXboxController(RobotConstants.Instance().kOperatorControllerPort);
    public final Superstructure kSuperstructure = new Superstructure(true, false, false, false);

    public RobotContainer() {
        configureBindings();
    }

    // Bind buttons to hardware.
    private void configureBindings() {
        kSuperstructure.bindDriveCommands(kDriveController);
    }
}