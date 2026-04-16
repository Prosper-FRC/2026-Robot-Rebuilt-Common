
package frc.robot.Auton;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.shooterState;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.Indexer.indexerState;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.intakePivotState;
import frc.robot.Subsystems.Intake.Intake.intakeRollerState;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

// TODO: Uncomment all of Arnav and Ragav's code once we have Choreo set up for testing.

public class AutonCommands {
    // private final AutoFactory kAutoFactory;

    private final Shooter kShooter;
    private final Indexer kIndexer;
    private final Intake kIntake;
    private final Drive kDrive;

    public AutonCommands(Drive drive, Shooter shooter, Intake intake, Indexer indexer) {
        kDrive = drive;

        // kAutoFactory = new AutoFactory(
        //     drive::getRobotPose,
        //     drive::resetOdometry,
        //     drive::followSwerveTrajectory,
        //     !RobotConstants.Instance().kIsBlueAlliance,
        //     drive);

        kShooter = shooter;
        kIntake = intake;
        kIndexer = indexer;
    }
    
    // public AutoRoutine LeftToHub() {

    //     AutoRoutine routine = kAutoFactory.newRoutine("Left to Hub");

    //     AutoTrajectory LeftToHub = routine.trajectory("fBLS_HS");
    //     AutoTrajectory OutOfTheWay = routine.trajectory("fHS_OOW");

    //     routine.active().onTrue(
    //         new SequentialCommandGroup(
    //             //kShooter.runShooterMotors(),
    //             LeftToHub.resetOdometry(),
    //             LeftToHub.cmd()
    //         )
    //     );

    //     LeftToHub.done().onTrue(
    //         new ParallelCommandGroup(
    //             //kIndexer.setIndexerStateCommand(IndexerState.Active),
    //             new WaitCommand(5.0)
    //         ).andThen(
    //             OutOfTheWay.cmd()
    //         )
    //     );

    //     return routine;
    // }

    // public AutoRoutine RightToHub() {

    //     AutoRoutine routine = kAutoFactory.newRoutine("Right to Hub");

    //     AutoTrajectory LeftToHub = routine.trajectory("fBRS_HS");
    //     AutoTrajectory OutOfTheWay = routine.trajectory("fHS_OOW");

    //     routine.active().onTrue(
    //         new SequentialCommandGroup(
    //             //kShooter.runShooterMotors(),
    //             LeftToHub.resetOdometry(),
    //             LeftToHub.cmd()
    //         )
    //     );

    //     LeftToHub.done().onTrue(
    //         new ParallelCommandGroup(
    //             //kIndexer.setIndexerStateCommand(IndexerState.Active),
    //             new WaitCommand(5.0)
    //         ).andThen(
    //             OutOfTheWay.cmd()
    //         )
    //     );

    //     return routine;
    // }

    // public AutoRoutine HubStart() {

    //     AutoRoutine routine = kAutoFactory.newRoutine("Hub Start");

    //     AutoTrajectory OutOfTheWay = routine.trajectory("fHS_OOW");

    //     routine.active().onTrue(
    //         new ParallelCommandGroup(
    //             //kShooter.runShooterMotors(),
    //             new WaitCommand(0.5)
    //         ).andThen(
    //             new ParallelCommandGroup(
    //             //kIndexer.setIndexerStateCommand(IndexerState.Active),
    //             new WaitCommand(5.0)
    //             ).andThen(
    //                 OutOfTheWay.cmd()
    //             )
    //         )
    //     );

    //     return routine;
    // }

    // public AutoRoutine testRoutine() // A test auto routine that moves the robot forward 1 meter and left 1 meter
    // {
    //     AutoRoutine routine = kAutoFactory.newRoutine("Test Routine");

    //     AutoTrajectory testPath = routine.trajectory("TestPath");
    //     AutoTrajectory testPath2 = routine.trajectory("TestPath2");

    //     routine.active().onTrue(
    //         new SequentialCommandGroup(
    //             testPath.resetOdometry(),
    //             testPath.cmd()
    //         )
    //     );

    //     testPath.done().onTrue(testPath2.cmd());

    //     return routine;
    // }

    public Command shooterAuto() {
        return new SequentialCommandGroup(kShooter.setShooterStateCommand(shooterState.Active),
        kIndexer.setIndexerStateCommand(indexerState.Active),
        kIntake.setIntakeStateCommand(intakePivotState.Deployed, intakeRollerState.Idling),
        new WaitCommand(5.0),
        kIntake.setIntakeRollerStateCommand(intakeRollerState.Inactive),
        kShooter.setShooterStateCommand(shooterState.Inactive),
        kIndexer.setIndexerStateCommand(indexerState.Inactive));
    }
}
