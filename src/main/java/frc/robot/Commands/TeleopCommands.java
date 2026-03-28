package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Superstructure.Superstructure;
import frc.robot.Superstructure.Superstructure.robotState;

public class TeleopCommands {
    private final Superstructure kSuperstructure;

    public TeleopCommands(Superstructure superstructure) {
        kSuperstructure = superstructure;
    }

    public Command setRobotState(robotState state) {
        return new InstantCommand(() -> { kSuperstructure.state = state; });
    }
}
