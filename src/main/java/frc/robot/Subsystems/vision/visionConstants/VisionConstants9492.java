package frc.robot.Subsystems.Vision.visionConstants;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants9492 extends VisionConstants{
   public VisionConstants9492() {
        kCamName = "limelight-lady";

        kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;
   }
}
