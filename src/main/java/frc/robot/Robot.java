// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Commands.AutonCommands;
import frc.robot.Subsystems.Drive.Drive;

public class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;
    
    private final AutonCommands kAutonCommands;
    private final AutoChooser kAutoChooser;

    public Robot() {
        // Sets up AK logging.
        switch(RobotConstants.Instance().kMode) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
            default:
                break;
        }

        Logger.start();

        m_robotContainer = new RobotContainer();

        kAutonCommands = new AutonCommands(m_robotContainer.getDrive(), m_robotContainer.getShooter(), m_robotContainer.getIndexer());
        kAutoChooser = new AutoChooser();

        kAutoChooser.addRoutine("Test Routine (NOT FOR COMP)", kAutonCommands::testRoutine);
        kAutoChooser.addRoutine("Simple Left to Hub", kAutonCommands::LeftToHub);
        kAutoChooser.addRoutine("Simple Right to Hub", kAutonCommands::RightToHub);
        kAutoChooser.addRoutine("Simple Hub Shoot", kAutonCommands::HubStart);

        RobotModeTriggers.autonomous().whileTrue(kAutoChooser.selectedCommandScheduler());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
