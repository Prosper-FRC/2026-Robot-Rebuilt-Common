package frc.robot.Subsystems.Shooter.ShooterConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class ShooterConstants{
    public final int kTeamNumber = RobotConstants.Instance().kTeamNumber;

    //Flywheel
    public int kFlywheelMotorID;

    public record FlywheelGains(
        double p, 
        double i, 
        double d, 
        double maxVelocityRadiansPerSecond, 
        double maxAccelerationRadiansPerSecondSquared, 
        double s, 
        double v, 
        double a, 
        double g
    ) {}

    public FlywheelGains kFlywheelGains;

    //Hood
    public int kHooderMotorID;
    public int kHooderCancoderID;
    public double kHooderArmLengthMeters = 0.15;
    public Rotation2d kHooderMaxAngleRads = new Rotation2d(Units.degreesToRadians(360.0));

    public record HooderGains(
        double p, 
        double i, 
        double d, 
        double maxVelocityRadiansPerSecond, 
        double maxAccelerationRadiansPerSecondSquared, 
        double s, 
        double v, 
        double a, 
        double g
    ) {}

    public HooderGains kHooderGains;

    //NEED TO CALCULATE
    public double kFlywheelVelocityTolerance = 10.0;
    public double kFlywheelVelocityOnRadiansPerSec = Units.rotationsPerMinuteToRadiansPerSecond(2700);
    public double kFlywheelVelocityOffRadiansPerSec = 0;

    public double kIndexerVelocityOnRadiansPerSec = Units.rotationsPerMinuteToRadiansPerSecond(800);
    public double kIndexerVelocityOffRadiansPerSec = 0;

    public Rotation2d kHoodPositionDefault = Rotation2d.fromDegrees(0.0);
    public Rotation2d kHoodPosition1 = Rotation2d.fromDegrees(90.0);
    public Rotation2d kHoodPosition2 = Rotation2d.fromDegrees(60.0);
    public Rotation2d kHoodPosition3 = Rotation2d.fromDegrees(30.0);

    public Pose2d kBlueHubPose = new Pose2d(4.6, 4, new Rotation2d(0));
    public Pose2d kRedHubPose = new Pose2d(11.9, 4, new Rotation2d(0));

    public ShooterConstants() {}
}