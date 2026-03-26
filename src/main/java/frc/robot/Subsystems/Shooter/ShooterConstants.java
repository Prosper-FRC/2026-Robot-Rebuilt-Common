package frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class ShooterConstants{
    public static ShooterConstants instance = null;
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
    public final double kFlywheelVelocityTolerance = 10.0;
    public final double kFlywheelVelocityOnRadiansPerSec = Units.rotationsPerMinuteToRadiansPerSecond(2000);
    public final double kFlywheelVelocityOffRadiansPerSec = 0;

    public final double kIndexerVelocityOnRadiansPerSec = Units.rotationsPerMinuteToRadiansPerSecond(800);
    public final double kIndexerVelocityOffRadiansPerSec = 0;

    public final Rotation2d kHoodPositionDefault = Rotation2d.fromDegrees(0.0);
    public final Rotation2d kHoodPosition1 = Rotation2d.fromDegrees(90.0);
    public final Rotation2d kHoodPosition2 = Rotation2d.fromDegrees(60.0);
    public final Rotation2d kHoodPosition3 = Rotation2d.fromDegrees(30.0);

    // Pose Estimates of Hubs
    public final Pose2d kBlueHubPose = new Pose2d(4.6, 4, new Rotation2d(0));
    public final Pose2d kRedHubPose = new Pose2d(11.9, 4, new Rotation2d(0));

    private ShooterConstants() {
        switch(kTeamNumber) {
            case 5411:
                kFlywheelGains = new FlywheelGains(
                    0.0,
                    0.0,
                    0.0,
                    Units.rotationsToRadians(2),
                    Units.rotationsToRadians(5),
                    0.0,
                    1.0,
                    1.0,
                    0.0
                );
                kHooderGains = new HooderGains(
                    0.0,
                    0.0,
                    0.0,
                    Units.rotationsToRadians(1),
                    Units.rotationsToRadians(3),
                    0.0,
                    5.0,
                    1.0,
                    0.0
                );
            case 9105:
                kFlywheelGains = new FlywheelGains(
                    0.0,
                    0.0,
                    0.0,
                    Units.rotationsToRadians(2),
                    Units.rotationsToRadians(5),
                    0.0,
                    1.0,
                    1.0,
                    0.0
                );
                kHooderGains = new HooderGains(
                    0.0,
                    0.0,
                    0.0,
                    Units.rotationsToRadians(1),
                    Units.rotationsToRadians(3),
                    0.0,
                    5.0,
                    1.0,
                    0.0
                );
            case 9492:
                kFlywheelGains = new FlywheelGains(
                    0.0,
                    0.0,
                    0.0,
                    Units.rotationsToRadians(2),
                    Units.rotationsToRadians(5),
                    0.0,
                    1.0,
                    1.0,
                    0.0
                );
                kHooderGains = new HooderGains(
                    0.0,
                    0.0,
                    0.0,
                    Units.rotationsToRadians(1),
                    Units.rotationsToRadians(3),
                    0.0,
                    5.0,
                    1.0,
                    0.0
                );
            case 0:
                kFlywheelMotorID = 1;
                kHooderMotorID = 2;
                kHooderCancoderID = 3;

                kFlywheelGains = new FlywheelGains(
                    2.0,
                    0.0,
                    0.0,
                    Units.rotationsPerMinuteToRadiansPerSecond(2500),
                    Units.rotationsPerMinuteToRadiansPerSecond(2000),
                    0.0,
                    0.02,
                    0.01,
                    0.0
                );
                kHooderGains = new HooderGains(
                    2.0,
                    0.1,
                    0.1,
                    Units.rotationsToRadians(1),
                    Units.rotationsToRadians(3),
                    0.0,
                    1.0,
                    1.0,
                    0.0
                );
                break;
        }
    }

    public static ShooterConstants getInstance() {
        if (instance == null) {
            instance = new ShooterConstants();

        }

        return instance;
    }

}