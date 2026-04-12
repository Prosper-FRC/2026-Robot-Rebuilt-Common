package frc.robot.Subsystems.Vision.VisionConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants {
    public String kCamName = "CAM_DEFAULT";

    public double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;

    // Nose going up is positive
    public final double kCamTiltDegrees = 30.234;


    public final Transform3d kCamTransform = new Transform3d(
        // Assuming the intake is on the front (of the robot) and the limelight is on the back in starting position
        new Translation3d(Units.inchesToMeters(11.962582), Units.inchesToMeters(-0.923649), Units.inchesToMeters(6.585075)),
        // Accounts for cameras being on back
        new Rotation3d(0.0, Units.degreesToRadians(30.234), 0.0)
    );

    public final Transform3d kBlueHubTop = new Transform3d(
        new Translation3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), Units.inchesToMeters(72.00)),
        new Rotation3d()
    );

    public final Transform3d kRedHubTop = new Transform3d(
        new Translation3d(Units.inchesToMeters(458.56), Units.inchesToMeters(158.84), Units.inchesToMeters(72.00)),
        new Rotation3d()
    );
    

    public VisionConstants() {}
}
