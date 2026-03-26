package frc.robot.Subsystems.Drive.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraConstants {
    public final AprilTagFieldLayout kLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public final Transform3d kCameraToRobotTranslation = new Transform3d();
    public final Rotation3d kCameraAngleRotation = new Rotation3d();
}
