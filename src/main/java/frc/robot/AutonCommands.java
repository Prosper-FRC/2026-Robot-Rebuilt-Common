
package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
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
    
    public AutoRoutine testRoutine() 
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

// crazy coding skills
/*
 * ok so a lot of stuff needs to be actually implemented but im hungry
 * also ragav i have no flippity flopping idea why the choreo folders are seperate ok
 * please delete the folder on your side or something idk i can fix this hopefully
 */