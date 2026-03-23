package frc.robot.Subsystems.Vision.visionConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants5411 extends VisionConstants{
    public static final String kCamName = "limelight-robo";

    public static final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;
    public VisionConstants5411() {

    }
}
