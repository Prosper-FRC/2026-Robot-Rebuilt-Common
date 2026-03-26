package frc.robot.Subsystems.Indexer.IndexerConstantsMain;

import frc.robot.Subsystems.Indexer.IndexerConstantsMain.IndexerConstants;

import frc.robot.RobotConstants;

public class IndexerConstants {   

    // Motor IDs
    public static int kHopperMotor1ID = 1;
    public static int kHopperMotor2ID = 2; 
    public static int kHopperMotor3ID = 3;
    public static int kIndexerMotor1ID = 4;

    public static IndexerConstants instance = null;

    public record IndexerTalonFXConfiguration(
        boolean invert,
        boolean enableStatorCurrentLimit,
        boolean enableSupplyCurrentLimit,
        double statorCurrentLimitsAmps,
        double supplyCurrentLimitsAmps,
        double peakForwardVoltage,
        double peakReverseVoltage) {}

    public record IndexerGains(
    // Feedback control
    double p, 
    double i, 
    double d, 
    // Motion magic constraints
    double maxVelocityMetersPerSecond, 
    double maxAccelerationMetersPerSecondSquared, 
    
    // Indexer feedforward values
    double s, 
    double v, 
    double a){}

   public static IndexerGains kHopperMotor2Gains; 
               
       static {
           switch (RobotConstants.Instance().kMode) {
                case REAL:
                    kHopperMotor2Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
                case SIM: 
                    //kHopperMotor2Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
                    break;
                default: 
                    kHopperMotor2Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
            }
        }

    public static IndexerGains kHopperMotor1Gains; 
            
    static {
        switch (RobotConstants.Instance().kMode) {
                case REAL:
                    kHopperMotor1Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
                case SIM: 
                    // kHopperMotor1Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
                    break;
                default: 
                    kHopperMotor1Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
            }
        }        

    public static IndexerGains kIndexerMotor1Gains; 
            
    static {
        switch (RobotConstants.Instance().kMode) {
                case REAL:
                    kIndexerMotor1Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
                case SIM: 
                    // kIndexerMotor1Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
                    break;
                default: 
                    kIndexerMotor1Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
            }
        }

    public static IndexerGains kHopperMotor3Gains; 
            
    static {
        switch (RobotConstants.Instance().kMode) {
                case REAL:
                    kHopperMotor3Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
                case SIM: 
                    //kHopperMotor3Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
                    break;
                default: 
                    kHopperMotor3Gains = new IndexerGains(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
            }
        }

    public static IndexerConstants getInstance() {
        if (instance == null) {
            instance = new IndexerConstants();
        }
        return instance;
    }
}