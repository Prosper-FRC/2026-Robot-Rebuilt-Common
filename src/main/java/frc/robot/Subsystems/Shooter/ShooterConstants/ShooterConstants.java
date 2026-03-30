package frc.robot.Subsystems.Shooter.ShooterConstants;

public class ShooterConstants {
    public static final record shooterGains(double kP, double kI, double kD, double kV) {}

    public static final record hoodGains(double kP, double kI, double kD) {}

    public final double maxShooterRPS = 60.0d;
    public final double maxHoodAngle = 0.0d;

    public final shooterGains kSGains = new shooterGains(0.15d, 0.0d, 0.0d, 0.12d);
    public final hoodGains kHGains = new hoodGains(0.3d, 0.0d, 0.0d);

    public final int flywheelID = 61;
    public final int hoodID = 62;
}
