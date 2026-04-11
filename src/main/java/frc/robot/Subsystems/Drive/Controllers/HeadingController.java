package frc.robot.Subsystems.Drive.Controllers;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;

public class HeadingController {
    private final PIDController kHeadingController;
    @AutoLogOutput(key = "Drive/HolonomicController/HeadingController/ThetaTargetMeters")
    private Rotation2d kHeadingGoal = new Rotation2d();

    public HeadingController() {
        kHeadingController = RobotConstants.DriveConstants().kHeadingController;
        kHeadingController.enableContinuousInput(-0.5d, 0.5d);
    }

    public void setHeadingGoal(Rotation2d angle) {
        kHeadingGoal = angle;
    }

    public Rotation2d calculateHeadingSetpoint(Rotation2d currentHeading) {
        return Rotation2d.fromRotations(kHeadingController.calculate(currentHeading.getRotations(), kHeadingGoal.getRotations()));
    }
}
