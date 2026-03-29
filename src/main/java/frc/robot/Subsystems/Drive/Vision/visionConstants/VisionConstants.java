
package frc.robot.Subsystems.Drive.Vision.VisionConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
// import frc.robot.subsystems.drive.DriveConstants;

public class VisionConstants {
    
    // From CAD and decided by you in configuration
    public String kCamName = "CAM_DEFAULT";

    public final Transform3d kCamTransform = new Transform3d(
        // Assuming the intake is on the front (of the robot) and the limelight is on the back in starting position
        new Translation3d(Units.inchesToMeters(11.962582), Units.inchesToMeters(-0.923649), Units.inchesToMeters(6.585075)),
        // Accounts for cameras being on back
        new Rotation3d(0.0, Units.degreesToRadians(30.234), 0.0)
    );

    // Nose going up is positive
    public final double kCamTiltDegrees = 30.234;

    public double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;

    public VisionConstants() {}
}