package frc.robot.Subsystems.Drive;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// DUMMY CLASS, DELETE BEFORE MERGING
// (Make sure the actual drive has all these methods implemented thou)
// I dont think the actual Drive.java has these so please make sure they are the same name

public class Drive extends SubsystemBase {
    
    private Pose2d odometeryPose;

    public Drive() {odometeryPose = new Pose2d();}

    public Pose2d getPose() {return odometeryPose;} // Gets the current odometery code

    public void resetOdometry(Pose2d pose) {odometeryPose = pose;} // Resets the odometery pose to the given pose

    public void followTrajectory(SwerveSample swerveSample) {} // Method that is used by Choreo to follow trajectories

}
