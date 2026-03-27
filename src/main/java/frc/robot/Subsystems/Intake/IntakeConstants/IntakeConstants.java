package frc.robot.Subsystems.Intake.IntakeConstants;

import edu.wpi.first.wpilibj.RobotController;

public class IntakeConstants {
    // Declare and Assign general constants here.
    public final double kRollerRPS = 45.0d;

    public record IntakeIDs(int PivotID, int RollerID, int CANCoderID) {}

    public record IntakeGains(
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

    public IntakeIDs kIntakeIDs = new IntakeIDs(0, 0, 0);
    public IntakeGains kPivotGains = new IntakeGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public IntakeGains kIntakeWheelsGains = new IntakeGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}