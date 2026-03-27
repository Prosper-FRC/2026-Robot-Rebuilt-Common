package frc.robot.Subsystems.Shooter.ShooterConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants5411 extends ShooterConstants {
    public HooderGains kHooderGains;
    
    public ShooterConstants5411() {    
        kFlywheelVelocityTolerance = 10.0;
        kFlywheelVelocityOnRadiansPerSec = Units.rotationsPerMinuteToRadiansPerSecond(2700);
        kFlywheelVelocityOffRadiansPerSec = 0;

        kIndexerVelocityOnRadiansPerSec = Units.rotationsPerMinuteToRadiansPerSecond(800);
        kIndexerVelocityOffRadiansPerSec = 0;

        kHoodPositionDefault = Rotation2d.fromDegrees(0.0);
        kHoodPosition1 = Rotation2d.fromDegrees(90.0);
        kHoodPosition2 = Rotation2d.fromDegrees(60.0);
        kHoodPosition3 = Rotation2d.fromDegrees(30.0);

        kBlueHubPose = new Pose2d(4.6, 4, new Rotation2d(0));
        kRedHubPose = new Pose2d(11.9, 4, new Rotation2d(0));
    
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
    }
}
