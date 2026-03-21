package frc.robot.Subsystems.Vision.visionConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants9492 extends VisionConstants{
    public static final String kCamName = "limelight-lady";

    public static final Transform3d kCamTransform = new Transform3d(
        new Translation3d(0.3, 0.0, 0.0),
        // Accounts for cameras being on back
        new Rotation3d(0.0, 0.0, 0.0)
    );

    public static final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;
    public VisionConstants9492() {}
}
