package frc.robot.Subsystems.Intake.IntakeConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants9105 extends IntakeConstants {
    public IntakeConstants9105() {
        kIntakePivotGains = new IntakeGains(new PIDGains(3.0, 0, 0.05), new FFGains(0, 0, 0, 0));
        kIntakeRollerGains = new IntakeGains(new PIDGains(0, 0, 0), new FFGains(0, 0, 0, 0));
        kIntakeIDs = new IntakeIDs(52, 51);
        kIntakeSoftLimits = new IntakeSoftLimits(12.0d, Rotation2d.fromRotations(0.0d), true, true);
        kIntakeHardLimits = new IntakeHardLimits(1.0d, 5.0d);
        rollerVoltageActive = 5.0d; // 10.0
        rollerVoltageIdle = 2.0d;
        pivotCurrentLimits = new IntakeCurrentLimits(60.0d, 40.0d);
        rollerCurrentLimits = new IntakeCurrentLimits(50.0d, 40.0d);
    }
}
