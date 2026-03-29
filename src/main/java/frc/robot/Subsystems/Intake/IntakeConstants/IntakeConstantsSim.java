package frc.robot.Subsystems.Intake.IntakeConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstantsSim extends IntakeConstants {
    public IntakeConstantsSim() {
        kIntakePivotGains = new IntakeGains(new PIDGains(35.0d, 0, 0), new FFGains(0, 0, 0, 0));
        kIntakeRollerGains = new IntakeGains(new PIDGains(0, 0, 0), new FFGains(0, 0, 0, 0));
        kIntakeIDs = new IntakeIDs(0, 0);
        kIntakeSoftLimits = new IntakeSoftLimits(12.0d, Rotation2d.fromRotations(0.25d), false, true);
        kIntakeHardLimits = new IntakeHardLimits(1.0d, 5.0d);
        targetVelocityRPM = 45.0d;
    }
}
