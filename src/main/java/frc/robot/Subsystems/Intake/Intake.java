package frc.robot.Subsystems.Intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Intake extends SubsystemBase {
    public static final Intake NoOp = new Intake(
        new RollerIO() {}, 
        new PivotIO() {});

    public static enum intakePivotState {
        Stowed(() -> 0.0d),
        Deployed(() -> 0.25d);
        private DoubleSupplier rotationSetpoint;

        private intakePivotState(DoubleSupplier setpoint) {
            rotationSetpoint = setpoint;
        }

        public DoubleSupplier getSetpoint() {
            return rotationSetpoint;
        }
    }

    public static enum intakeRollerState {
        Inactive,
        Intake,
        Idling,
        Outtake
    }

    @AutoLogOutput(key = "Intake/Pivot/State")
    public intakePivotState pivotState = intakePivotState.Stowed;

    @AutoLogOutput(key = "Intake/Roller/State")
    public intakeRollerState rollerState = intakeRollerState.Inactive;

    private final RollerIO kRoller;
    private final PivotIO kPivot;

    private final RollerInputsAutoLogged kRollerInputs = new RollerInputsAutoLogged();
    private final PivotInputsAutoLogged kPivotInputs = new PivotInputsAutoLogged();

    private final Debouncer kIntakeDeployDebouncer = new Debouncer(0.150);

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

    public void setIntakePivotState(intakePivotState state) {
        pivotState = state;
    }
    public void setIntakeRollerState(intakeRollerState state) {
        rollerState = state;
    }

    public Command setIntakePivotStateCommand(intakePivotState state) {
        return new InstantCommand(() -> setIntakePivotState(state), this);
    }
    public Command setIntakeRollerStateCommand(intakeRollerState state) {
        return new InstantCommand(() -> setIntakeRollerState(state), this);
    }

    public Command setIntakeStateCommand(intakePivotState pState, intakeRollerState rState) {
        return new InstantCommand(() -> {
            setIntakePivotState(pState);
            setIntakeRollerState(rState);
        });
    }

    public boolean isAtGoal() {
        return kIntakeDeployDebouncer.calculate(Math.abs(kPivotInputs.positionRotations - pivotState.getSetpoint().getAsDouble()) <= 0.05);
    }

    @Override
    public void periodic() {
        kRoller.updateInputs(kRollerInputs);
        kPivot.updateInputs(kPivotInputs);

        Logger.processInputs("Intake/Roller", kRollerInputs);
        Logger.processInputs("Intake/Pivot", kPivotInputs);

        kPivot.setTargetPosition(Rotation2d.fromRotations(pivotState.getSetpoint().getAsDouble()));

        switch (rollerState) {
            case Inactive:
                kRoller.stopRollerMotor();
                break;
            case Intake:
                kRoller.setOutputVoltage(RobotConstants.IntakeConstants().rollerVoltageActive);
                break;
            case Idling:
                kRoller.setOutputVoltage(RobotConstants.IntakeConstants().rollerVoltageIdle);
                break;
            case Outtake:
                kRoller.setOutputVoltage(-RobotConstants.IntakeConstants().rollerVoltageActive);
                break;
            default:
                break;
        }
    }
}
