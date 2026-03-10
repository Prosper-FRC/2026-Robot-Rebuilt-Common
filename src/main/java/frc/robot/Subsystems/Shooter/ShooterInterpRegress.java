package frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterInterpRegress extends SubsystemBase {

    public final static InterpolatingDoubleTreeMap distanceToAngleMap = new InterpolatingDoubleTreeMap();
    
    private DoubleSupplier kDistanceFromTarget;

    public static void ShooterTreeMap() 
    //uncomment "static" to test interpolation, and replace null values with data
    //static
    {
        distanceToAngleMap.put(null, null); //put experimental data- key is distance and value is corresponding angle
        distanceToAngleMap.put(null, null);
        distanceToAngleMap.put(null, null);
        distanceToAngleMap.put(null, null);
    }

    // Dummy estimate 
    public static final Supplier<Pose2d> kGetPoseEstimate = () -> { return new Pose2d(); };

    public static Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(distanceToAngleMap.get(kGetPoseEstimate.get().getRotation().getDegrees()));
    }

   

    //uncomment to test interpolation

    // public static Rotation2d getAngle(double distance1)
    // {
    //     double interpolatedAngle1 = distanceToAngleMap.get(distance1);
    //     return Rotation2d.fromDegrees(interpolatedAngle1);
    // }

    private final FlywheelIO kFlywheel;
    private final FlywheelInputsAutoLogged kFlywheelInputs;

    private final HooderIO kHooder;
    private final HooderInputsAutoLogged kHooderInputs;

    public enum HooderPosition {
        kHoodPosition1(ShooterConstants.getInstance().kHoodPosition1),
        kHoodPosition2(ShooterConstants.getInstance().kHoodPosition2),
        kHoodPosition3(ShooterConstants.getInstance().kHoodPosition3);

        public final Rotation2d angle;

        HooderPosition(Rotation2d angle) {
            this.angle = angle;
        }
    }

    @AutoLogOutput(key = "Flywheel/FlywheelOn")
    private boolean flywheelOn = false;

    @AutoLogOutput(key = "Flywheel/FlywheelGoalRPM")
    private double flywheelGoalRPM = 0.0;

    private HooderPosition currentHooderPosition = HooderPosition.kHoodPosition1;

    public ShooterInterpRegress(FlywheelIO flywheelIO, HooderIO hooderIO) {
        kFlywheel = flywheelIO;
        kFlywheelInputs = new FlywheelInputsAutoLogged();

        kHooder = hooderIO;
        kHooderInputs = new HooderInputsAutoLogged();
    }

    @Override
    public void periodic() {
        kFlywheel.updateInputs(kFlywheelInputs);
        kHooder.updateInputs(kHooderInputs);

        Logger.processInputs("Flywheel", kFlywheelInputs);
        Logger.processInputs("Hooder", kHooderInputs);

        Logger.recordOutput("Hooder/HooderPosition", currentHooderPosition);
    }

    // Flywheel IO Functions
    public void setFlywheelVoltage(double volts) {
        kFlywheel.setFlywheelVoltage(volts);
    }

    public void setFlywheelVelocity(double velocity) {
        kFlywheel.setFlywheelVelocity(velocity);
    }

    public void stopFlywheel() {
        kFlywheel.stopFlywheel();
    }

    public void resetFlywheel() {
        kFlywheel.resetFlywheel();
    }

    // Hood IO Functions

    public void setHooderVoltage(double volts) {
        kHooder.setHooderVoltage(volts);
    }
    
    public void setHooderPositionRotationsGoal(Rotation2d newHoodPosition) {
        kHooder.setHooderPositionRotationsGoal(newHoodPosition);
    }

    public void stopHooder() {
        kHooder.stopHooder();
    }
    
    public void resetHooder() {
        kHooder.resetHooder();
    }

    public double getHooderPositionRadiansGoal() {
        return kHooder.getHooderPositionRadiansGoal();
    }

    // Hood
    public void nextPosition() {
        int newIndex = currentHooderPosition.ordinal() + 1;
        HooderPosition[] values = HooderPosition.values();
        
        if (newIndex < values.length) {setHoodPosition(values[newIndex]);}
    }

    public void previousPosition() {
        int newIndex = currentHooderPosition.ordinal() - 1;
        HooderPosition[] values = HooderPosition.values();
        
        if (newIndex >= 0) {setHoodPosition(values[newIndex]);}
    }

    public void setHoodPosition(HooderPosition newPos) {
        currentHooderPosition = newPos;
        setHooderPositionRotationsGoal(newPos.angle);
    }

    public void flywheelOnOff() {
        if (flywheelOn) {
            kFlywheel.stopFlywheel();
            flywheelGoalRPM = (ShooterConstants.getInstance().kFlywheelVelocityOffRadiansPerSec * 60) / (2 * Math.PI);
            flywheelOn = false;
        } else {
            kFlywheel.setFlywheelVelocity(ShooterConstants.getInstance().kFlywheelVelocityOnRadiansPerSec);
            flywheelOn = true;
        }
    }
}
