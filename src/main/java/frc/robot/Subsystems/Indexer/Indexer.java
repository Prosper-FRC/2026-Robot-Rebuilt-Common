package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    public static final Indexer NoOp = new Indexer(
        new HopperIO() {},
        new HopperIO() {},
        new HopperIO() {},
        new HopperIO() {}
    );

    private final HopperIO[] kHopperMotors = new HopperIO[4];
    private final hopperInputsAutoLogged[] kHopperInputs = new hopperInputsAutoLogged[] {
        new hopperInputsAutoLogged(),
        new hopperInputsAutoLogged(),
        new hopperInputsAutoLogged(),
        new hopperInputsAutoLogged()
    };

    public Indexer(HopperIO hopper1, HopperIO hopper2, HopperIO hopper3, HopperIO ballTunnel) {
        kHopperMotors[0] = hopper1;
        kHopperMotors[1] = hopper2;
        kHopperMotors[2] = hopper3;
        kHopperMotors[3] = ballTunnel;
    }

    public void setHoppers(double voltage) {
        kHopperMotors[0].setHopperVoltage(voltage);
        kHopperMotors[1].setHopperVoltage(voltage);
        kHopperMotors[2].setHopperVoltage(voltage);
    }
    public Command setHoppersCommand(double voltage) {
        return new InstantCommand(() -> setHoppers(voltage), this);
    }

    public void stopHoppers() {
        kHopperMotors[0].stopHopper();
        kHopperMotors[1].stopHopper();
        kHopperMotors[2].stopHopper();
    }
    public Command stopHoppersCommand() {
        return new InstantCommand(() -> stopHoppers(), this);
    }

    public Command setHoppersAndBallTunnelCommand(double hopperVolts, double tunnelVolts) {
        return new RunCommand(() -> {
            setHoppers(hopperVolts);
            setBallTunnel(tunnelVolts);
        });
    }
    public Command stopHoppersAndBallTunnelCommand() {
        return new InstantCommand(() -> {
            stopHoppers();
            stopBallTunnel();
        });
    }

    public void setBallTunnel(double voltage) {
        kHopperMotors[3].setHopperVoltage(voltage);    
    }
    public Command setBallTunnelCommand(double voltage) {
        return new InstantCommand(() -> setBallTunnel(voltage));
    }

    public void stopBallTunnel() {
        kHopperMotors[3].stopHopper();
    }
    public Command stopBallTunnelCommand() {
        return new InstantCommand(() -> stopBallTunnel(), this);
    }

    @Override
    public void periodic() {
        kHopperMotors[0].updateInputs(kHopperInputs[0]);
        kHopperMotors[1].updateInputs(kHopperInputs[1]);
        kHopperMotors[2].updateInputs(kHopperInputs[2]);
        kHopperMotors[3].updateInputs(kHopperInputs[3]);

        Logger.processInputs("Indexer/Hopper1", kHopperInputs[0]);
        Logger.processInputs("Indexer/Hopper2", kHopperInputs[1]);
        Logger.processInputs("Indexer/Hopper3", kHopperInputs[2]);
        Logger.processInputs("Indexer/BallTunnel", kHopperInputs[3]);
    }
}
