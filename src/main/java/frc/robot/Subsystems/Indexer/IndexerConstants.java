package frc.robot.Subsystems.Indexer;

import frc.robot.RobotConstants;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotController;

public class IndexerConstants {

    public static IndexerConstants instance = null;
    public final int kTeamNumber;

    // Motor IDs
    public int kHopperMotor1ID;
    public int kHopperMotor2ID;
    public int kHopperMotor3ID;
    public int kIndexerMotor1ID;

    public static IndexerConstants instants = null;

    public record IndexerTalonFXConfiguration(
        boolean invert,
        boolean enableStatorCurrentLimit,
        boolean enableSupplyCurrentLimit,
        double statorCurrentLimitsAmps,
        double supplyCurrentLimitsAmps,
        double peakForwardVoltage,
        double peakReverseVoltage) {}

    private IndexerConstants() {
        // hardware line?
        kTeamNumber = RobotController.getTeamNumber();
        switch (kTeamNumber) {
            case 5411:
                //Motors IDs for 5411 bot
                break;
            case 9105:
                //Motors IDs for 9105 bot
                break;
            case 9492:
                //Motors IDs for 9492 bot
                break;
            case 0: //sim
                kHopperMotor1ID = 1;
                kHopperMotor2ID = 2;
                kHopperMotor3ID = 3;
                kIndexerMotor1ID = 4;
                break;
        }
    }

    public static IndexerConstants getInstance() {
        if (instance == null) {
            instance = new IndexerConstants();
        }
        return instance;
    }
}