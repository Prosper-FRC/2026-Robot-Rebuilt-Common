
package frc.robot;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Drive.Drive;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

// imports

public class AutonCommands {

    private final AutoFactory autoFactory;

    public AutonCommands(Drive drive) {

        autoFactory = new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drive // The drive subsystem
        );
    }
    
    public AutoRoutine testRoutine() // A test auto routine that moves the robot forward 1 meter and left 1 meter
    {
        AutoRoutine routine = autoFactory.newRoutine("Test Routine");

        AutoTrajectory testPath = routine.trajectory("TestPath");
        AutoTrajectory testPath2 = routine.trajectory("TestPath2");

        routine.active().onTrue(
            new SequentialCommandGroup(
                testPath.resetOdometry(),
                testPath.cmd()
            )
        );

        testPath.done().onTrue(testPath2.cmd());

        return routine;
    }

}