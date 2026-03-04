package frc.robot.Subsystems.Intake.IntakeConstants;

import edu.wpi.first.wpilibj.RobotController;

public class IntakeConstants {
    private static IntakeConstants instance = null;

    // Declare and Assign general constants here.
    public final double kRollerRPS = 67.0d;

    public record PivotGains(
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
    double a,
    double g){}

    public record IntakeWheelsGains(
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


   

    public static IntakeConstants getInstance() {
        // Using a null check so that the instance is created at the proper time.
        if (instance == null) {
            instance = new IntakeConstants();
        }
        return instance;
    }
}