package frc.robot.Subsystems.Shooter;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        kHoodPosition1(() -> ShooterConstants.getInstance().kHoodPosition1),
        kHoodPosition2(() -> ShooterConstants.getInstance().kHoodPosition2),
        kHoodPosition3(() -> ShooterConstants.getInstance().kHoodPosition3),
        kHoodPositionDefault(() -> ShooterConstants.getInstance().kHoodPositionDefault),
        kHoodPositionAuto(() -> getAngle(getDistanceFromHub()));

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
    
    @AutoLogOutput(key = "Hooder/DistanceFromTarget")
    private DoubleSupplier kDistanceFromTarget = () -> getDistanceFromHub();

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
        System.out.println(kDistanceFromTarget.getAsDouble());

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

    // Hood
    // public void nextPosition() {
    //     int newIndex = currentHooderPosition.ordinal() + 1;
    //     HooderPosition[] values = HooderPosition.values();
        
    //     if (newIndex < values.length) {setHoodPosition(values[newIndex]);}
    // }

    // public void previousPosition() {
    //     int newIndex = currentHooderPosition.ordinal() - 1;
    //     HooderPosition[] values = HooderPosition.values();
        
    //     if (newIndex >= 0) {setHoodPosition(values[newIndex]);}
    // }

    // public void setHoodPosition(HooderPosition newPos) {
    //     currentHooderPosition = newPos;
    //     setHooderPositionRotationsGoal(newPos.angle);
    // }

    // Get distance from hub
    public static double getDistanceFromHub() {
        Optional<Alliance> team = DriverStation.getAlliance();
        
        // TODO: CHANGE THE WAY POSE IS GOTTEN ONCE SUPERSTRUCTURE IS DONE
        Translation2d robotPosition = RobotContainer.getPoseEstimate.get().getTranslation();

        Translation2d blueHub = ShooterConstants.getInstance().kBlueHubPose.getTranslation();
        Translation2d redHub = ShooterConstants.getInstance().kRedHubPose.getTranslation();
        
        // Distance from blue hub
        if (team.equals(Alliance.valueOf("Blue"))) {
            if (robotPosition.getX() > blueHub.getX()) {
                return (-robotPosition.getDistance(blueHub));
            } else {
                return robotPosition.getDistance(blueHub);
            }
        } 
        // Distance from red hub
        else if (team.equals(Alliance.valueOf("Red"))) {
            if (robotPosition.getX() < redHub.getX()) {
                return (-robotPosition.getDistance(redHub));
            } else {
                return robotPosition.getDistance(redHub);
            }
        } 
        // Distance from either hub (SIM)
        else {
            double distanceToBlueHub = robotPosition.getDistance(blueHub);
            double distanceToRedHub = robotPosition.getDistance(redHub);
            double closestDistance = distanceToBlueHub < distanceToRedHub ? distanceToBlueHub : distanceToRedHub;

            if ((robotPosition.getX() < redHub.getX()) && (robotPosition.getX() > blueHub.getX())) {
                return -closestDistance;
            } else {
                return closestDistance;
            }
        }
    }

    public static Rotation2d getAngle(double distance)
    {
        return Rotation2d.fromDegrees(distanceToAngleMap.get(distance));
    }

    public void setHoodPosition(HooderPosition newPosition) {
        this.state = newPosition;
    }

    // To set a certain set point based off distance
    public void setHoodSetpoint() {
        double distance = getDistanceFromHub();
        if ((distance > 0) && (distance <= 2)) {
            this.state = state.kHoodPosition1;
        } else if ((distance > 2) && (distance <= 4)) {
            this.state = state.kHoodPosition2;
        } else {
            this.state = state.kHoodPosition3;
        }
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
