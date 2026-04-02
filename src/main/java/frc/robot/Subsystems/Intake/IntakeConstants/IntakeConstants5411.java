package frc.robot.Subsystems.Intake.IntakeConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants5411 extends IntakeConstants {
    public IntakeConstants5411() {
        kIntakePivotGains = new IntakeGains(new PIDGains(0, 0, 0), new FFGains(0, 0, 0, 0));
        kIntakeRollerGains = new IntakeGains(new PIDGains(0, 0, 0), new FFGains(0, 0, 0, 0));
        kIntakeIDs = new IntakeIDs(0, 0);
        kIntakeSoftLimits = new IntakeSoftLimits(12.0d, Rotation2d.fromRotations(0.0d), false, true);
        kIntakeHardLimits = new IntakeHardLimits(1.0d, 5.0d);
        rollerVoltageActive = 10.0d;
        rollerVoltageIdle = 2.0d;
        pivotCurrentLimits = new IntakeCurrentLimits(60.0d, 40.0d);
        rollerCurrentLimits = new IntakeCurrentLimits(50.0d, 40.0d);
    }
}
