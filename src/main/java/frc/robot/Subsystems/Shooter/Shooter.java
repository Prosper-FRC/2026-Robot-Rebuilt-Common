package frc.robot.Subsystems.Shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public static final Shooter NoOp = new Shooter(new FlywheelIO() {}, new HoodIO() {});

    public static enum shooterHoodSetpoint {
        Hub(() -> 0.0d);

        private DoubleSupplier setpoint;

        private shooterHoodSetpoint(DoubleSupplier setpoint) {
            this.setpoint = setpoint;
        }

        public DoubleSupplier getSetpoint() {
            return setpoint;
        }
    }
    public shooterHoodSetpoint setpoint = shooterHoodSetpoint.Hub;

    private final FlywheelIO kFlywheel;
    private final HoodIO kHood;

    private final flywheelInputsAutoLogged kFlywheelInputs = new flywheelInputsAutoLogged();
    private final hoodInputsAutoLogged kHoodInputs = new hoodInputsAutoLogged();
    
    public Shooter(FlywheelIO flywheel, HoodIO hood) {
        kFlywheel = flywheel;
        kHood = hood;
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

    @Override
    public void periodic() {
        kFlywheel.updateInputs(kFlywheelInputs);
        kHood.updateInputs(kHoodInputs);
        Logger.processInputs("Shooter/Flywheel", kFlywheelInputs);
        Logger.processInputs("Shooter/Hood", kHoodInputs);
    }
}
