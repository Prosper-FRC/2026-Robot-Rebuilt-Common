package frc.robot.Subsystems.Shooter.ShooterConstants;

public class ShooterConstants {
    public static final record shooterGains(double kP, double kI, double kD, double kV) {}

    public static final record hoodGains(double kP, double kI, double kD) {}

    public static final record currentLimits(double statorCurrent, double supplyCurrent) {}

    public final double kShooterRPS = 55.0d;

    public shooterGains kShooterGains;
    public hoodGains kHGains;

    public final currentLimits kFlywheelCurrentLimits = new currentLimits(100.0d, 55.0d); 

    public final int flywheelID = 61;
    public final int hoodID = 62;
}
