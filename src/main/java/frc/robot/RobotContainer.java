// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeSim;
import frc.robot.Subsystems.Intake.Intake.IntakeState;

public class RobotContainer {
    public final Intake kIntake = new Intake(new IntakeSim());
    public final CommandXboxController kController = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        kController.a().onTrue(
            new InstantCommand(() -> kIntake.setIntakeState(IntakeState.Deployed))
        ).whileFalse(
            new InstantCommand(() -> kIntake.setIntakeState(IntakeState.Stowed))
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
