package frc.robot.Subsystems.Shooter.ShooterConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterConstants {
    public static final record shooterGains(double kP, double kI, double kD, double kV) {}
    public static final record currentLimits(double statorCurrent, double supplyCurrent) {}

    public final double kShooterRPS = 53.0d;

    public final shooterGains kShooterGains = new shooterGains(0.1d, 0.0d, 0.0d, 0.15d);

    public final currentLimits kFlywheelCurrentLimits = new currentLimits(100.0d, 55.0d); 

    public final int flywheelID = 61;

    public final PIDController kSimFlywheelController = new PIDController(1.0d, 0.0d, 0.0d);
    public final SimpleMotorFeedforward kSimFlywheelFeedforward = new SimpleMotorFeedforward(0.0d, 0.11d);
}
