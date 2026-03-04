package frc.robot.Subsystems.Intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants;

public class Intake extends SubsystemBase {
    public static enum IntakeState {
        Stowed(() -> 0.0d), // Using dummy poses at the moment
        Deployed(() -> 0.5d);

        private final DoubleSupplier kGoalPose;

        private IntakeState(DoubleSupplier goalPose) {
            kGoalPose = goalPose;
        }

        public double getGoalPoseRotations() {
            return kGoalPose.getAsDouble();
        }
    }

    private final IntakeIO kIntake;
    private final IntakeInputsAutoLogged kInputs = new IntakeInputsAutoLogged();

    @AutoLogOutput(key = "Intake/State")
    public IntakeState state = IntakeState.Stowed;

    public Intake(IntakeIO hardware) {
        kIntake = hardware;
    }

    public void setIntakeState(IntakeState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        kIntake.updateInputs(kInputs);
        Logger.processInputs("Intake", kInputs);

        switch (state) {
            case Stowed:
                kIntake.setPivotPositionRotations(state.getGoalPoseRotations());
                kIntake.stopRoller();;
                break;
            case Deployed:
                kIntake.setPivotPositionRotations(state.getGoalPoseRotations());
                kIntake.setRollerSpeedRPS(IntakeConstants.getInstance().kRollerRPS);; // Motion magic lets this work just fine.
                break;
            default:
                break;
        }
    }
}
