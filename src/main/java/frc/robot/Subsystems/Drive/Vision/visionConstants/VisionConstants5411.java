package frc.robot.Subsystems.Drive.Vision.visionConstants;

import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants5411 extends VisionConstants{
    public VisionConstants5411() {
        kCamName = "limelight-robo";
        kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;
    }
}
