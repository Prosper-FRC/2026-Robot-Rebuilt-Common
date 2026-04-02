package frc.robot.Subsystems.Shooter.ShooterConstants;

public class ShooterConstantsSim extends ShooterConstants {
    public ShooterConstantsSim() {
        kShooterGains = new shooterGains(0.1d, 0.0d, 0.0d, 0.12d);
        kHGains = new hoodGains(0.3d, 0.0d, 0.0d);
    }   
}
