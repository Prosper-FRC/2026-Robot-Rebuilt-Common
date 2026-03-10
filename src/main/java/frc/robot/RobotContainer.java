// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Indexer.IndexerConstants.*;
import frc.robot.Subsystems.Indexer.IndexerTalonFX.*;
import frc.robot.Subsystems.Indexer.IndexerIO.*;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerIO;
import frc.robot.Subsystems.Indexer.IndexerTalonFX;
import frc.robot.Subsystems.Indexer.Indexer.*;

public class RobotContainer {

    private final Indexer indexer;

    private final CommandXboxController driver = new CommandXboxController(0);

    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {

        indexer = new Indexer(new IndexerTalonFX());
        configureBindings();
    }

    private void configureBindings() {
        driver.a().onTrue(
        indexer.setIndexerStateCommand(Indexer.IndexerState.Active)
    );

    driver.b().onTrue(
        indexer.setIndexerStateCommand(Indexer.IndexerState.Idle)
    );

    driver.x().onTrue(
        indexer.setIndexerStateCommand(Indexer.IndexerState.Eject)
    );
    }
}
