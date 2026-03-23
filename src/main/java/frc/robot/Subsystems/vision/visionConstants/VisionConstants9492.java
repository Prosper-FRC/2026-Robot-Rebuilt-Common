package frc.robot.Subsystems.Vision.visionConstants;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants9492 extends VisionConstants{
    public static final String kCamName = "limelight-lady";

    public static final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;
    public VisionConstants9492() {}
}
