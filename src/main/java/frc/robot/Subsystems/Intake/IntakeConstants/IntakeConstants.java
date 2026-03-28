package frc.robot.Subsystems.Intake.IntakeConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
    public static final record PIDGains(double kP, double kI, double kD) {}
    public static final record FFGains(double kV, double kS, double kA, double kG) {}
    
    public static final record IntakeGains(PIDGains pidGains, FFGains feedForwardGains) {}
    public static final record IntakeIDs(int pivotID, int rollerID) {}
    
    public static final record IntakeSoftLimits(double voltageLimits, Rotation2d rangeOfMotion, boolean isInverted, boolean isBraked) {}
    public static final record IntakeHardLimits(double rollerGearRatio, double pivotGearRatio) {}

    public IntakeGains kIntakePivotGains;
    public IntakeGains kIntakeRollerGains;
    public IntakeIDs kIntakeIDs;
    public IntakeSoftLimits kIntakeSoftLimits;
    public IntakeHardLimits kIntakeHardLimits;
    public double targetVelocityRPM;
}
