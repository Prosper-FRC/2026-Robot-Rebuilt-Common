
package frc.robot.Subsystems.Vision.visionConstants;



import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
// import frc.robot.subsystems.drive.DriveConstants;

public class VisionConstants {
    
    // From CAD and decided by you in configuration
    public final String kCamName = "CAM_DEFAULT";

    public final Transform3d kCamTransform = new Transform3d(
        // Assuming the intake is on the front (of the robot) and the limelight is on the back in starting position
        new Translation3d(Units.inchesToMeters(12.499), Units.inchesToMeters(-0.604), Units.inchesToMeters(7.361)),
        // Accounts for cameras being on back
        new Rotation3d(0.0, Units.degreesToRadians(30.234), 0.0)
    );

    // Nose going up is positive
    public final double kCamTiltDegrees = 30.234;

    public final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.5 : 1.0;

    public VisionConstants() {}
}