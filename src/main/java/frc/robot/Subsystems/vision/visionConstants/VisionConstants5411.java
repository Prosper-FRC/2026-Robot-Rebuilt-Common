package frc.robot.Subsystems.Vision.visionConstants;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants5411 extends VisionConstants{
    public final String kCamName = "limelight-robo";

    public final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.5 : 1.0;
    public VisionConstants5411() {

    }
}
