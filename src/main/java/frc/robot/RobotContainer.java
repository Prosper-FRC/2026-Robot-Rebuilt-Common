// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.FlywheelIOSim;
import frc.robot.Subsystems.Shooter.HooderIOSim;
import frc.robot.Subsystems.Shooter.ShooterConstants;

public class RobotContainer {
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.getInstance().kDriveControllerPort);
    public Shooter kShooter;

    public static Supplier<Pose2d> getPoseEstimate;

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

                // TESTING POSE
                getPoseEstimate = () -> { return new Pose2d(); };

                // UNCOMMENT FOR DRIVE SIM
                // getPoseEstimate = () -> { return kDrive.getOdometryPose().get(); };
                break;
            default:
                break;
        }

    	configureBindings();
  	}

	private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
        
        // Hooder
        kDriveController.a().onTrue(
            Commands.runOnce(() -> kShooter.hoodAutoOn = !kShooter.hoodAutoOn, kShooter)
        );

        kDriveController.x().onTrue(
            Commands.runOnce(() -> kShooter.setHoodSetpoint(), kShooter)
        );
    
        // Flywheel
        kDriveController.b().onTrue(
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