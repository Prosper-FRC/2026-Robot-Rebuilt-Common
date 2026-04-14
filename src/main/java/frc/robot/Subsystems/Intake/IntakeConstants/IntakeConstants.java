package frc.robot.Subsystems.Intake.IntakeConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
    public static final record PIDGains(double kP, double kI, double kD) {}
    public static final record FFGains(double kV, double kS, double kA, double kG) {}
    
    public static final record IntakeGains(PIDGains pidGains, FFGains feedForwardGains) {}
    public static final record IntakeIDs(int pivotID, int rollerID) {}
    
    public static final record IntakeSoftLimits(double voltageLimits, Rotation2d rangeOfMotion, boolean isInverted, boolean isBraked) {}
    public static final record IntakeHardLimits(double rollerGearRatio, double pivotGearRatio) {}

    public static final record IntakeCurrentLimits(double statorCurrent, double supplyCurrent) {}

    public final IntakeGains kIntakePivotGains = new IntakeGains(new PIDGains(2.0, 0, 0), new FFGains(0, 0, 0, 0));
    public final IntakeGains kIntakeRollerGains = new IntakeGains(new PIDGains(0, 0, 0), new FFGains(0, 0, 0, 0));
    public final IntakeIDs kIntakeIDs = new IntakeIDs(52, 51);
    public final IntakeSoftLimits kIntakeSoftLimits = new IntakeSoftLimits(12.0d, Rotation2d.fromRotations(0.0d), true, true);
    public final IntakeHardLimits kIntakeHardLimits = new IntakeHardLimits(1.0d, 5.0d);
    public final double rollerVoltageActive = 5.0d;
    public final double rollerVoltageIdle = 2.0d;
    public final IntakeCurrentLimits pivotCurrentLimits = new IntakeCurrentLimits(60.0d, 40.0d);
    public final IntakeCurrentLimits rollerCurrentLimits  = new IntakeCurrentLimits(50.0d, 40.0d);
    
    public final PIDController kSimIntakePID = new PIDController(35.0d, 0.0d, 0.0d);
}
