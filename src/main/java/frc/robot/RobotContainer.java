// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Subsystems.Vision.LimelightIO;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.visionConstants.VisionConstants;


public class RobotContainer {
    private final Vision vision;

    private final VisionConstants vConstants = RobotConstants.getInstance().getVisionConstants();
    private final String kCamName = vConstants.kCamName;
    private final Transform3d kCamTransform = vConstants.kCamTransform;

    public RobotContainer() {
        configureBindings();
        switch (RobotConstants.getInstance().kMode) {
           case REAL:
                vision = new Vision(new LimelightIO(kCamName, kCamTransform));
                break;
            case REPLAY:
                vision = new Vision(new LimelightIO(kCamName, kCamTransform));
                break;
            case SIM:
                vision = new Vision(new LimelightIO(kCamName, kCamTransform));
                break;
            default:
                vision = new Vision(new LimelightIO(kCamName, kCamTransform));
                break;
        }
    }

    private void configureBindings() {}
    
    
}
