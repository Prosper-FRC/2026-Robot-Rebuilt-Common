// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.Gyro.GyroSim;
import frc.robot.Subsystems.Drive.SwerveModule.SwerveModuleSim;
import frc.robot.Subsystems.Drive.SwerveModule.SwerveModuleTalonFX;
import frc.robot.Subsystems.Drive.SwerveSetpointGenerator.SetpointGenerator;
import frc.robot.Subsystems.Drive.SwerveSetpointGenerator.SwerveConfiguration;

public class RobotContainer {
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.Instance().kDriveControllerPort);
    public final CommandXboxController kOperatorController = new CommandXboxController(RobotConstants.Instance().kOperatorControllerPort);
    
    public final Drive kDrive;

    public RobotContainer() {
        kDrive = new Drive(
            new SwerveModuleTalonFX(
                RobotConstants.DriveConstants().kFLModuleIds,
                RobotConstants.DriveConstants().kFLModuleOffset,
                RobotConstants.DriveConstants().kCanbus
            ), 
            new SwerveModuleTalonFX(
                RobotConstants.DriveConstants().kFRModuleIds,
                RobotConstants.DriveConstants().kFRModuleOffset,
                RobotConstants.DriveConstants().kCanbus
            ), 
            new SwerveModuleTalonFX(
                RobotConstants.DriveConstants().kBLModuleIds,
                RobotConstants.DriveConstants().kBLModuleOffset,
                RobotConstants.DriveConstants().kCanbus
            ), 
            new SwerveModuleTalonFX(
                RobotConstants.DriveConstants().kBRModuleIds,
                RobotConstants.DriveConstants().kBRModuleOffset,
                RobotConstants.DriveConstants().kCanbus
            ), 
            new GyroSim()
        );
        SetpointGenerator generator = new SetpointGenerator(new SwerveConfiguration());

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        kDrive.supplyControllerInputs(
            () -> kDriveController.getLeftX(), 
            () -> kDriveController.getLeftY(), 
            () -> kDriveController.getRightX()
        );
    }
}