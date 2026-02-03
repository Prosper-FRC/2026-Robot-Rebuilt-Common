package frc.robot.Subsystems.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static enum IntakeState {
        Stowed(() -> 0.0d), // Using dummy poses at the moment
        Deployed(() -> 0.0d);

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

        switch (state) {
            case Stowed:
                kIntake.setPivotPositionRotations(state.getGoalPoseRotations());
                kIntake.setRollerSpeedRPS(IntakeConstants.getInstance().kRollerRPS);
                break;
            case Deployed:
                kIntake.setPivotPositionRotations(state.getGoalPoseRotations());
                kIntake.stopRoller(); // The roller should probably be set to coast so it free spins for some time, or else a delay will need to be set for the roller to slow down.
                break;
            default:
                break;
        }
    }
}
