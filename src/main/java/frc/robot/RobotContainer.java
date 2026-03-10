// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Drive.Drive;

public class RobotContainer {

    private final Drive drive;

    private final AutonCommands autonCommands;

    public RobotContainer() {

        drive = new Drive();

        autonCommands = new AutonCommands(drive);

        configureBindings();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

    private void configureBindings() {}

}
