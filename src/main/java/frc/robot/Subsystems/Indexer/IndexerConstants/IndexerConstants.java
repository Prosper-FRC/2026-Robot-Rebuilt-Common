package frc.robot.Subsystems.Indexer.IndexerConstants;

import frc.robot.RobotConstants;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotController;

public class IndexerConstants {

    public static IndexerConstants instance = null;
   

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

    public record Indexer1Gains(
    // Feedback control
    double p, 
    double i, 
    double d, 
    // Motion magic constraints
    double maxVelocityMetersPerSecond, 
    double maxAccelerationMetersPerSecondSquared, 
    
    // Climb feedforward values
    double s, 
    double v, 
    double a){}


    
    public record Indexer2Gains(
    // Feedback control
    double p, 
    double i, 
    double d, 
    // Motion magic constraints
    double maxVelocityMetersPerSecond, 
    double maxAccelerationMetersPerSecondSquared, 
    
    // Climb feedforward values
    double s, 
    double v, 
    double a){}




    public record Indexer3Gains(
    // Feedback control
    double p, 
    double i, 
    double d, 
    // Motion magic constraints
    double maxVelocityMetersPerSecond, 
    double maxAccelerationMetersPerSecondSquared, 
    
    // Climb feedforward values
    double s, 
    double v, 
    double a){}



    public record Indexer4Gains(
    // Feedback control
    double p, 
    double i, 
    double d, 
    // Motion magic constraints
    double maxVelocityMetersPerSecond, 
    double maxAccelerationMetersPerSecondSquared, 
    
    // Climb feedforward values
    double s, 
    double v, 
    double a){}






    

    public static IndexerConstants getInstance() {
        if (instance == null) {
            instance = new IndexerConstants();
        }
        return instance;
    }
}