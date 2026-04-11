package frc.robot.Subsystems.Drive.Controllers;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotConstants;

public class HolonomicController {
    public final PIDController kTranslationalXController;
    @AutoLogOutput(key = "Drive/HolonomicController/XTargetMeters")
    public double setpointXMeters = 0.0d;
    public final PIDController kTranslationalYController;
    @AutoLogOutput(key = "Drive/HolonomicController/YTargetMeters")
    public double setpointYMeters = 0.0d;
    public final HeadingController kHeadingController;

    public HolonomicController() {
        kTranslationalXController = RobotConstants.DriveConstants().kXTranslationalController;
        kTranslationalYController = RobotConstants.DriveConstants().kYTranslationalController;
        kHeadingController = new HeadingController();
    }

    public void setDesiredFieldPose(Pose2d pose) {
        setpointXMeters = pose.getX();
        setpointYMeters = pose.getY();
        kHeadingController.setHeadingGoal(pose.getRotation());
    }
    public void setDesiredFieldPose(double xMeters, double yMeters, Rotation2d angle) {
        setpointXMeters = xMeters;
        setpointYMeters = yMeters;
        kHeadingController.setHeadingGoal(angle);
    }
    public void setDesiredHeading(Rotation2d angle) {
        kHeadingController.setHeadingGoal(angle);
    }

    public ChassisSpeeds calculatePositionSetpoint(double xReadingMeters, double yReadingMeters, Rotation2d chassisAngle, ChassisSpeeds feedforward) {
        return calculateSetpoint(xReadingMeters, yReadingMeters, chassisAngle, feedforward);
    }
    public ChassisSpeeds calculatePositionSetpoint(Pose2d positionReading, ChassisSpeeds feedforward) {
        return calculateSetpoint(positionReading.getX(), positionReading.getY(), positionReading.getRotation(), feedforward);
    }
    public Rotation2d calculateHeadingSetpoint(Rotation2d currentAngle) {
        return kHeadingController.calculateHeadingSetpoint(currentAngle);
    }

    private ChassisSpeeds calculateSetpoint(double x, double y, Rotation2d theta, ChassisSpeeds feedforward) {
        return new ChassisSpeeds(
            feedforward.vxMetersPerSecond + kTranslationalXController.calculate(x, setpointXMeters),
            feedforward.vyMetersPerSecond + kTranslationalYController.calculate(y, setpointYMeters),
            kHeadingController.calculateHeadingSetpoint(theta).getRadians()
        );
    }
}
