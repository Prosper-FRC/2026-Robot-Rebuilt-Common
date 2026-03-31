
package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Superstructure.Superstructure;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

// imports

public class AutonCommands {

    private final AutoFactory kAutoFactory;
    //private final Superstructure kSuperstructure;

    private final Shooter kShooter;
    private final Indexer kIndexer;

    public AutonCommands(Drive drive, Shooter shooter, Indexer indexer) {

        kAutoFactory = new AutoFactory(
            drive::getRobotPose,
            drive::resetOdometry,
            drive::folllowTrajectoryChor,
            false, // TODO: get the alliance here somehow
            drive);

        kShooter = shooter;
        kIndexer = indexer;
    }
    
    public AutoRoutine LeftToHub() {

        AutoRoutine routine = kAutoFactory.newRoutine("Left to Hub");

        AutoTrajectory LeftToHub = routine.trajectory("fBLS_HS");
        AutoTrajectory OutOfTheWay = routine.trajectory("fHS_OOW");

        routine.active().onTrue(
            new SequentialCommandGroup(
                kShooter.setShooterCommand(0, 0), // TODO: get actual values
                LeftToHub.resetOdometry(),
                LeftToHub.cmd()
            )
        );

        LeftToHub.done().onTrue(
            new ParallelCommandGroup(
                kIndexer.setHoppersAndBallTunnelCommand(0, 0), // TODO: get actual values
                new WaitCommand(5.0)
            ).andThen(
                OutOfTheWay.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine RightToHub() {

        AutoRoutine routine = kAutoFactory.newRoutine("Right to Hub");

        AutoTrajectory LeftToHub = routine.trajectory("fBRS_HS");
        AutoTrajectory OutOfTheWay = routine.trajectory("fHS_OOW");

        routine.active().onTrue(
            new SequentialCommandGroup(
                kShooter.setShooterCommand(0, 0),
                LeftToHub.resetOdometry(),
                LeftToHub.cmd()
            )
        );

        LeftToHub.done().onTrue(
            new ParallelCommandGroup(
                kIndexer.setHoppersAndBallTunnelCommand(0, 0),
                new WaitCommand(5.0)
            ).andThen(
                OutOfTheWay.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine HubStart() {

        AutoRoutine routine = kAutoFactory.newRoutine("Hub Start");

        AutoTrajectory OutOfTheWay = routine.trajectory("fHS_OOW");

        routine.active().onTrue(
            new ParallelCommandGroup(
                kShooter.setShooterCommand(0, 0),
                new WaitCommand(0.5)
            ).andThen(
                new ParallelCommandGroup(
                kIndexer.setHoppersAndBallTunnelCommand(0, 0),
                new WaitCommand(5.0)
                ).andThen(
                    OutOfTheWay.cmd()
                )
            )
        );

        return routine;
    }

    public AutoRoutine testRoutine() // A test auto routine that moves the robot forward 1 meter and left 1 meter
    {
        AutoRoutine routine = kAutoFactory.newRoutine("Test Routine");

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