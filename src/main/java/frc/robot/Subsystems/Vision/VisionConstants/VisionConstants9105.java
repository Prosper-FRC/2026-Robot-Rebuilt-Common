package frc.robot.Subsystems.Vision.VisionConstants;

import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants9105 extends VisionConstants{
    public VisionConstants9105() {
        kCamName = "limelight-techno";
        kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;
    }
}
