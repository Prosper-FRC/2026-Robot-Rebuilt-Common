package frc.robot.Factories;

import frc.robot.RobotConstants;
import frc.robot.Subsystems.Indexer.HopperSim;
import frc.robot.Subsystems.Indexer.HopperTalonFX;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerConstants.IndexerConstants;

public class IndexerFactory {
    private static final IndexerConstants kConstants = RobotConstants.IndexerConstants();

    private IndexerFactory() {}

    public static Indexer createReal() {
        return new Indexer(
            new HopperTalonFX(kConstants.kHopper1Id, kConstants.kHopper1Inverted),
            new HopperTalonFX(kConstants.kHopper2Id, kConstants.kHopper2Inverted),
            new HopperTalonFX(kConstants.kHopper3Id, kConstants.kHopper3Inverted),
            new HopperTalonFX(kConstants.kBallTunnelId, kConstants.kBallTunnelInverted)
        );
    }
    
    public static Indexer createSim() {
        return new Indexer(
            new HopperSim(),
            new HopperSim(),
            new HopperSim(),
            new HopperSim()
        );
    }

    public static Indexer createNoOp() {
        return Indexer.NoOp;
    }
}
