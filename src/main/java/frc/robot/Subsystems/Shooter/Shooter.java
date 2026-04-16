package frc.robot.Subsystems.Shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Shooter extends SubsystemBase {
    public static final Shooter NoOp = new Shooter(new FlywheelIO() {}, new HoodIO() {});

    public static enum shooterState {
        Inactive, 
        Active;
    }
    @AutoLogOutput(key = "Shooter/State")
    public shooterState state = shooterState.Inactive;

    private final FlywheelIO kFlywheel;
    private final HoodIO kHood;

    private final flywheelInputsAutoLogged kFlywheelInputs = new flywheelInputsAutoLogged();
    private final hoodInputsAutoLogged kHoodInputs = new hoodInputsAutoLogged();

    private final Debouncer kShooterSpeedDebouncer = new Debouncer(0.15d);

    private double hoodPosition = 0.0d;

    private DoubleSupplier hoodAxis = () -> 0.0d;
    
    public Shooter(FlywheelIO flywheel, HoodIO hood) {
        kFlywheel = flywheel;
        kHood = hood;
    }

    public void supplyHoodAxis(DoubleSupplier supplier) {
        hoodAxis = supplier;
    }

    public void setShooter(double rps, double position) {
        kFlywheel.setFlywheelRPS(rps);
        kHood.setHoodPosition(position);
    }
    public void setFlywheelVolts(double volts) {
        kFlywheel.setFlywheelVoltage(volts);
    }
    public void setHoodVolts(double volts) {
        kHood.setHoodVoltage(volts);
    }
    public void stopShooter() {
        kFlywheel.stopFlywheel();
    }
    public void setShooterState(shooterState state) {
        this.state = state;
    }

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
    public Command overrideHoodPositionCommand(double position) {
        return new InstantCommand(() -> hoodPosition = position);
    }

    public boolean isShooterSpunUp() {
        return kShooterSpeedDebouncer.calculate(Math.abs(RobotConstants.ShooterConstants().kShooterRPS - kFlywheelInputs.velocityRPS) < 10.0d);
    }

    @Override
    public void periodic() {
        kFlywheel.updateInputs(kFlywheelInputs);
        kHood.updateInputs(kHoodInputs);
        Logger.processInputs("Shooter/Flywheel", kFlywheelInputs);
        Logger.processInputs("Shooter/Hood", kHoodInputs);

        double readGoal = hoodAxis.getAsDouble();

        if(Math.abs(readGoal) > 0.1d) {
            hoodPosition += MathUtil.copyDirectionPow(readGoal, 2.0d) * RobotConstants.Instance().kTimestep * RobotConstants.ShooterConstants().kHoodMaxAngularVelocityRPS;
        }

        kHood.setHoodPosition(hoodPosition);

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
