package frc.robot.Factories;

import frc.robot.RobotConstants;
import frc.robot.Factories.SubsystemFactory.SubsystemType;
import frc.robot.Subsystems.Indexer.HopperSim;
import frc.robot.Subsystems.Indexer.HopperTalonFX;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerConstants.IndexerConstants;

public class IndexerFactory {
    public static final IndexerConstants kIndexerConstants = RobotConstants.IndexerConstants();

    public static Indexer create(SubsystemType type) {
        switch(type) {
            case REAL:
                return new Indexer(
                    new HopperTalonFX(kIndexerConstants.kHopper1Id, kIndexerConstants.kHopper1Inverted), 
                    new HopperTalonFX(kIndexerConstants.kHopper2Id, kIndexerConstants.kHopper2Inverted), 
                    new HopperTalonFX(kIndexerConstants.kHopper3Id, kIndexerConstants.kHopper3Inverted),
                    new HopperTalonFX(kIndexerConstants.kBallTunnelId, kIndexerConstants.kBallTunnelInverted)
                );
            case SIM:
                return new Indexer(
                    new HopperSim(), 
                    new HopperSim(), 
                    new HopperSim(),
                    new HopperSim()
                );
            default:
                return Indexer.NoOp;
        }
    }
}
