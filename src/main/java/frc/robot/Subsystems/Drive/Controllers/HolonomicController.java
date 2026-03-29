package frc.robot.Subsystems.Drive.Controllers;

import org.littletonrobotics.junction.AutoLogOutput;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// This is the top layer of the Cascading PID controller used in Choreo path following, this allows us to account for translational drift in the robots position.
public class HolonomicController {
    private final ProfiledPIDController kXPositionController;
    private final ProfiledPIDController kYPositionController;

    @AutoLogOutput(key = "Drive/HolonomicController/TargetSetpoint")
    private Pose2d targetPoseSetpoint = new Pose2d();
    
    @AutoLogOutput(key = "Drive/HolonomicController/velocityGoalSpeeds")
    private ChassisSpeeds velocityGoal = new ChassisSpeeds();

    public HolonomicController(ProfiledPIDController xPositionController, ProfiledPIDController yPositionController) {
        kXPositionController = xPositionController;
        kYPositionController = yPositionController;
    }

    // Gets the updated velocity goals with the input.
    public ChassisSpeeds getVelocityGoal(ChassisSpeeds feedforwardVelocity, Pose2d robotPoseTarget, Pose2d robotPoseEstimation) {
        velocityGoal = new ChassisSpeeds(
            feedforwardVelocity.vxMetersPerSecond, 
            feedforwardVelocity.vyMetersPerSecond, 
            0.0d); // Feedforward velocity is returned by Choreo.
        
        // Update the velocity goal with the translational error's offset.
        velocityGoal.vxMetersPerSecond += kXPositionController.calculate(robotPoseEstimation.getX(), robotPoseTarget.getX());
        velocityGoal.vyMetersPerSecond += kYPositionController.calculate(robotPoseEstimation.getY(), robotPoseTarget.getY());
        return velocityGoal; // Made field relative in the state logic handling in Drive.java.
    }

    public void followTrajectory(SwerveSample sample) {
        velocityGoal = sample.getChassisSpeeds();
    } 
}
