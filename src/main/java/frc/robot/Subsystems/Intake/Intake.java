package frc.robot.Subsystems.Intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public static final Intake NoOp = new Intake(
        new RollerIO() {}, 
        new PivotIO() {});

    public static enum intakeState {
        Stowed(() -> 0.0d),
        Deployed(() -> 0.0d);

        private DoubleSupplier rotationSetpoint;

        private intakeState(DoubleSupplier setpoint) {
            rotationSetpoint = setpoint;
        }

        public DoubleSupplier getSetpoint() {
            return rotationSetpoint;
        }
    }

    public intakeState state = intakeState.Stowed;

    private final RollerIO kRoller;
    private final PivotIO kPivot;

    private final RollerInputsAutoLogged kRollerInputs = new RollerInputsAutoLogged();
    private final PivotInputsAutoLogged kPivotInputs = new PivotInputsAutoLogged();

    public Intake(RollerIO rollerHardware, PivotIO pivotHardware) {
        kRoller = rollerHardware;
        kPivot = pivotHardware;
    }

    // COMMANDS
    public Command setPivotVoltageCommand(double voltage) {
        return new InstantCommand(() -> kPivot.setOutputVoltage(voltage), this);
    }

    public Command setRollerVoltageCommand(double voltage) {
        return new InstantCommand(() -> kRoller.setOutputVoltage(voltage), this);
    }

    public Command setPivotPositionCommand(Rotation2d position) {
        return new InstantCommand(() -> kPivot.setTargetPosition(position), this);
    }

    public Command setRollerVelocityCommand(Rotation2d velocityPerSecond) {
        return new InstantCommand(() -> kRoller.setTargetVelocity(velocityPerSecond), this);
    }

    public Command stopPivotCommand() {
        return new InstantCommand(() -> kPivot.stopPivotMotor(), this);
    }

    public Command stopRollerCommand() {
        return new InstantCommand(() -> kRoller.stopRollerMotor(), this);
    }

    public Command resetPivotCommand() {
        return new InstantCommand(() -> kPivot.resetPivotMotor(), this);
    }

    public void setIntakeState(intakeState state) {
        this.state = state;
    }

    public Command setIntakeStateCommand(intakeState state) {
        return new InstantCommand(() -> setIntakeState(state), this);
    }

    @Override
    public void periodic() {
        kRoller.updateInputs(kRollerInputs);
        kPivot.updateInputs(kPivotInputs);

        Logger.processInputs("Intake/Roller", kRollerInputs);
        Logger.processInputs("Intake/Pivot", kPivotInputs);

        setPivotPositionCommand(Rotation2d.fromRotations(state.getSetpoint().getAsDouble()));
        switch (state) {
            case Deployed:
                setRollerVelocityCommand(Rotation2d.fromRotations(45.0d));
                break;
            default:
                stopRollerCommand();
                break;
        }
    }
}
