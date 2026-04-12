package frc.robot.Subsystems.Drive.Controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class HeadingController {
    private PIDController aimController;
    
    public HeadingController() {
        aimController = new PIDController(0.5, 0, 0);
        aimController.enableContinuousInput(-0.5d, 0.5d);
        aimController.setTolerance(1.0);

    }

    // Return angular velocity needed to get to goal yaw/rotation
    public double AimDownSights(Rotation2d currentRotation, Rotation2d goalRotation) {
        double pidOutput = aimController.calculate(currentRotation.getRadians(), goalRotation.getRadians());
        return pidOutput;
    }
}
