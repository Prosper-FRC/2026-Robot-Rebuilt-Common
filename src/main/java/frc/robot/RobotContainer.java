// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.FlywheelIOSim;
import frc.robot.Subsystems.Shooter.HooderIOSim;
import frc.robot.Subsystems.Shooter.ShooterConstants;

public class RobotContainer {
  public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.getInstance().kDriveControllerPort);
  public Shooter kShooter;

  public RobotContainer() {
    	switch (RobotConstants.getInstance().kCurrentMode) {
          case REAL:
                //STUFF
                break;
            case REPLAY:
                break;
            case SIM:
                kShooter = new Shooter(
                    new FlywheelIOSim(
                        ShooterConstants.getInstance().kFlywheelGains
                    ), 
                    new HooderIOSim(
                        ShooterConstants.getInstance().kHooderGains
                    )
                );
                break;
            default:
                break;
        }

    	configureBindings();
  	}

	private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
		
        // Hooder
        kDriveController.b().onTrue(
            Commands.runOnce(() -> kShooter.setHoodPosition(Shooter.HooderPosition.kHoodPosition1), kShooter)
);
    
        // Flywheel
        kDriveController.a().onTrue(
            Commands.runOnce(() -> kShooter.flywheelOnOff(), kShooter)
        );
     
        kDriveController.start().onTrue(
            Commands.runOnce(() -> {
                kShooter.stopFlywheel();
                kShooter.stopHooder();
            }, kShooter)
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

}