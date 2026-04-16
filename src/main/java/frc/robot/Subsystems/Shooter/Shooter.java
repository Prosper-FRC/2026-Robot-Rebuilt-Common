package frc.robot.Subsystems.Shooter;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotConstants;

public class Shooter extends SubsystemBase {
    public static final Shooter NoOp = new Shooter(new FlywheelIO() {});

    public static enum shooterState {
        Inactive, 
        Active;
    }
    @AutoLogOutput(key = "Shooter/State")
    public shooterState state = shooterState.Inactive;

    @AutoLogOutput(key = "Shooter/IsAtFullSpeed")
    public boolean isAtFullSpeed = false;

    private final FlywheelIO kFlywheel;

    private final flywheelInputsAutoLogged kFlywheelInputs = new flywheelInputsAutoLogged();

    private final Debouncer kShooterSpeedDebouncer = new Debouncer(0.15d);
    
    public Shooter(FlywheelIO flywheel) {
        kFlywheel = flywheel;
    }

    /********** Util Methods **********/
    public void setShooter(double rps, double position) {
        kFlywheel.setFlywheelRPS(rps);
    }
    public void setFlywheelVolts(double volts) {
        kFlywheel.setFlywheelVoltage(volts);
    }
    public void setHoodVolts(double volts) {
    }
    public void stopShooter() {
        kFlywheel.stopFlywheel();
    }
    public void setShooterState(shooterState state) {
        this.state = state;
    }
    public boolean isShooterFullSpeed() {
        return kShooterSpeedDebouncer.calculate(Math.abs(RobotConstants.ShooterConstants().kShooterRPS - kFlywheelInputs.velocityRPS) < 5.0d);
    }

    /********** COMMANDS **********/
    public Command setShooterCommand(double rps, double position) {
        return new RunCommand(() -> setShooter(rps, position));
    }
    public Command setFlywheelVoltsCommand(double volts) {
        return new InstantCommand(() -> setFlywheelVolts(volts));
    } 
    public Command setHoodVoltsCommand(double volts) {
        return new InstantCommand(() -> setHoodVolts(volts));
    }
    public Command stopShooterCommand() {
        return new InstantCommand(() -> stopShooter());
    }
    public Command setShooterStateCommand(shooterState state) {
        return new InstantCommand(() -> setShooterState(state));
    }
    public Trigger isShooterFullSpeedTrigger() {
        return new Trigger(() -> isShooterFullSpeed());
    }

    /********** State Machine Periodic **********/
    @Override
    public void periodic() {
        kFlywheel.updateInputs(kFlywheelInputs);
        Logger.processInputs("Shooter/Flywheel", kFlywheelInputs);
        
        isAtFullSpeed = isShooterFullSpeed();

        switch (state) {
            case Active:
                kFlywheel.setFlywheelRPS(RobotConstants.ShooterConstants().kShooterRPS);
                break;
            case Inactive:
                kFlywheel.stopFlywheel();
                break;
            default:
                break;
        }
    }
}
