package frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;

import java.lang.Thread.State;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final FlywheelIO kFlywheel;
    private final FlywheelInputsAutoLogged kFlywheelInputs;

    private final HooderIO kHooder;
    private final HooderInputsAutoLogged kHooderInputs;

    public enum HooderPosition {
        kHoodPosition1(() -> ShooterConstants.getInstance().kHoodPosition1),
        kHoodPosition2(() -> ShooterConstants.getInstance().kHoodPosition2),
        kHoodPosition3(() -> ShooterConstants.getInstance().kHoodPosition3),
        kHoodPositionAuto(() -> ShooterConstants.getInstance().kHoodPositionAuto);

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
    public HooderPosition state = HooderPosition.kHoodPosition1;

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

        switch(state) {
            case kHoodPosition1:
                setHooderPositionRotationsGoal(state.getGoalPosition());
                break;
            case kHoodPosition2:
                setHooderPositionRotationsGoal(state.getGoalPosition());
                break;
            case kHoodPosition3:
                setHooderPositionRotationsGoal(state.getGoalPosition());
                break;
            case kHoodPositionAuto:
                setHooderPositionRotationsGoal(state.getGoalPosition());
                break;
            default:
                stopFlywheel();
                stopHooder();
                break;

        }
        
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

    // Hood
    // public void nextPosition() {
    //     int newIndex = state.ordinal() + 1;
    //     HooderPosition[] values = HooderPosition.values();
        
    //     if (newIndex < values.length) {setHoodPosition(values[newIndex]);}
    // }

    // public void previousPosition() {
    //     int newIndex = state.ordinal() - 1;
    //     HooderPosition[] values = HooderPosition.values();
        
    //     if (newIndex >= 0) {setHoodPosition(values[newIndex]);}
    // }
    //Add stuff to RoboContainer
    /* 
    public void setHoodPosition(HooderPosition newPos) {
        state = newPos;
        setHooderPositionRotationsGoal(newPos.goalPosition);
    }
    */

    public void setHoodPosition(HooderPosition newPosition) {
        this.state = newPosition;
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
