
package frc.robot.Subsystems.vision.visionConstants;



import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.wpilibj.RobotBase;
// import frc.robot.subsystems.drive.DriveConstants;

public class VisionConstants {
    
    // From CAD and decided by you in configuration
    public static final String kCamName = "CAM_DEFAULT";

    public static final Transform3d kCamTransform = new Transform3d(
        new Translation3d(0.3, 0.0, 0.0),
        // Accounts for cameras being on back
        new Rotation3d(0.0, 0.0, 0.0)
    );

    public static final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;

    public VisionConstants() {}
}