package frc.robot.Subsystems.Vision.visionConstants;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants9105 extends VisionConstants{
    public final String kCamName = "limelight-techno";

    public final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;
    public VisionConstants9105() {}
}
