package frc.robot.Subsystems.Indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;

public class HopperSim implements HopperIO {
    private double appliedVoltage = 0.0d;
    
    private final DCMotorSim kHopperMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.005d, RobotConstants.IndexerConstants().kIndexerGearing), 
        DCMotor.getKrakenX60(1),
        0.0d, 0.0d);
    
    public HopperSim() {}

    @Override
    public void updateInputs(hopperInputs toUpdate) {
        toUpdate.isOk = true;
        toUpdate.hopperRPS = kHopperMotor.getAngularVelocityRPM() / 60.0d;
        toUpdate.hopperVoltage = kHopperMotor.getInputVoltage();
        toUpdate.hopperStatorCurrent = -1.0d;
        toUpdate.hopperSupplyCurrent = kHopperMotor.getCurrentDrawAmps();

        kHopperMotor.setInputVoltage(appliedVoltage);

        kHopperMotor.update(RobotConstants.Instance().kTimestep);
    }

    @Override
    public void setHopperVoltage(double volts) {
        appliedVoltage = volts;
    }

    @Override
    public void stopHopper() {
        appliedVoltage = 0.0d;
    }
}
