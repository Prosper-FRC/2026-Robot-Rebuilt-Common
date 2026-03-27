package frc.robot.Subsystems.Shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Shooter extends SubsystemBase {
    public final static InterpolatingDoubleTreeMap distanceToAngleMap = new InterpolatingDoubleTreeMap();

    // public void ShooterTreeMap() 
    static
    {
        distanceToAngleMap.put(-30.0, 90.0);
        distanceToAngleMap.put(-20.0, 60.0);
        distanceToAngleMap.put(-10.0, 30.0);
        distanceToAngleMap.put(0.0 , 0.0);
        distanceToAngleMap.put(10.0, 30.0);
        distanceToAngleMap.put(20.0, 60.0);
        distanceToAngleMap.put(30.0, 90.0);
    }

    private final FlywheelIO kFlywheel;
    private final FlywheelInputsAutoLogged kFlywheelInputs;

    private final HooderIO kHooder;
    private final HooderInputsAutoLogged kHooderInputs;

    public enum HooderPosition {
        kHoodPosition1(() -> RobotConstants.ShooterConstants().kHoodPosition1),
        kHoodPosition2(() -> RobotConstants.ShooterConstants().kHoodPosition2),
        kHoodPosition3(() -> RobotConstants.ShooterConstants().kHoodPosition3),
        kHoodPositionDefault(() -> RobotConstants.ShooterConstants().kHoodPositionDefault),
        kHoodPositionAuto(() -> new Rotation2d());

        public Supplier<Rotation2d> goalPosition;

        HooderPosition(Supplier<Rotation2d> goalPosition) {
            this.goalPosition = goalPosition;
        }

        public Rotation2d getGoalPosition() {
            return this.goalPosition.get();
        }

        public void setGoalPosition(Supplier<Rotation2d> goalPosition) {
            this.goalPosition = goalPosition;
        }
    }

    @AutoLogOutput(key = "Flywheel/FlywheelOn")
    private boolean flywheelOn = false;
    
    @AutoLogOutput(key = "Flywheel/FlywheelGoalRPM")
    private double flywheelGoalRPM = 0.0;

    @AutoLogOutput(key = "Hooder/State")
    public HooderPosition state = HooderPosition.kHoodPositionAuto;
    
    @AutoLogOutput(key = "Hooder/HoodAutoOn")
    public boolean hoodAutoOn = false;

    public Shooter(FlywheelIO flywheelIO, HooderIO hooderIO) {
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

        Logger.recordOutput("Hooder/HooderPosition", state);

        if (state == HooderPosition.kHoodPositionAuto) {
            if(hoodAutoOn) {
                setHooderPositionRotationsGoal(state.getGoalPosition());
            }      
        } else if (state == HooderPosition.kHoodPosition1 || state == HooderPosition.kHoodPosition2 || state == HooderPosition.kHoodPosition3 || state == HooderPosition.kHoodPositionDefault) {
            setHooderPositionRotationsGoal(state.getGoalPosition());   
        }

        // switch(state) {
        //     case kHoodPosition1:
        //         setHooderPositionRotationsGoal(state.getGoalPosition());
        //         break;
        //     case kHoodPosition2:
        //         setHooderPositionRotationsGoal(state.getGoalPosition());
        //         break;
        //     case kHoodPosition3:
        //         setHooderPositionRotationsGoal(state.getGoalPosition());
        //         break;
        //     case kHoodPositionAuto:
        //         if (hoodAutoOn) {
        //             setHooderPositionRotationsGoal(state.getGoalPosition());
        //         }
        //         break;
        //     case kHoodPositionDefault:
        //         setHooderPositionRotationsGoal(state.getGoalPosition());
        //         break;
        //     default:
        //         stopFlywheel();
        //         stopHooder();
        //         break;
        // }
        
        if (DriverStation.isDisabled()) {
            stopHooder();
            stopFlywheel();
        }
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

    public static Rotation2d getAngle(double distance)
    {
        return Rotation2d.fromDegrees(distanceToAngleMap.get(distance));
    }

    public void setHoodPosition(HooderPosition newPosition) {
        this.state = newPosition;
    }

    public void flywheelOnOff() {
        if (flywheelOn) {
            kFlywheel.stopFlywheel();
            flywheelGoalRPM = (RobotConstants.ShooterConstants().kFlywheelVelocityOffRadiansPerSec * 60) / (2 * Math.PI);
            flywheelOn = false;
        } else {
            kFlywheel.setFlywheelVelocity(RobotConstants.ShooterConstants().kFlywheelVelocityOnRadiansPerSec);
            flywheelOn = true;
        }
    }
}
