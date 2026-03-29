package frc.robot.Subsystems.Drive.Controllers;

import java.security.Timestamp;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;

public class HeadingController {
    private Supplier<Rotation2d> kGyroAngle;
    private final ProfiledPIDController kHeadingController;

    @AutoLogOutput(key = "Drive/HeadingController/DesiredSetpoint")
    private double angleSetpoint = 0.0d;

    @AutoLogOutput(key = "Drive/HeadingController/DesiredOmega")
    private Rotation2d omegaOuput = Rotation2d.kZero;

    public HeadingController(ProfiledPIDController headingController) {
        kHeadingController = headingController;
        kHeadingController.enableContinuousInput(-0.5d, 0.5d);
    }

    public void supplyGyroAngle(Supplier<Rotation2d> gyroAngle) {
        kGyroAngle = gyroAngle;
    }

    public void setHeading(Rotation2d angle) {
        angleSetpoint = angle.getRotations();
    }
    public void offsetHeading(Rotation2d omega) {
        angleSetpoint += omega.getRotations() * RobotConstants.Instance().kTimestep;
        
    }

    /**
     * Calculates the desired omega output of the robot chassis in order to reach the setpoint. Does not use a feedforward value in the setpoint calculation.
     * @return the desired velocity setpoint to reach the desired angle.
     */
    public Rotation2d getOmega() {
        double angleRotations = kGyroAngle.get().getRotations();
        double angleFeedforward = 0.0d;
        omegaOuput = Rotation2d.fromRotations(kHeadingController.calculate(kGyroAngle.get().getRotations(), angleSetpoint) + angleFeedforward);
        return omegaOuput;
    }
    /**
     * Calculates the desired omega output of the robot chassis in order to reach the setpoint. Uses a supplied feedforward value in the setpoint calculation.
     * @param angleFeedforward The feedforward value in rotations per second to add to the output.
     * @return the desired velocity setpoint to reach the desired angle.
     */
    public Rotation2d getOmega(Rotation2d angleFeedforward) {
        double angleRotations = kGyroAngle.get().getRotations();
        omegaOuput = Rotation2d.fromRotations(kHeadingController.calculate(kGyroAngle.get().getRotations(), angleSetpoint) + angleFeedforward.getRotations());
        return omegaOuput;
    }
}
