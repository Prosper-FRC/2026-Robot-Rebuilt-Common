package frc.robot.Factories;

import frc.robot.RobotConstants;
import frc.robot.Factories.SubsystemFactory.SubsystemType;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.PivotSim;
import frc.robot.Subsystems.Intake.PivotTalonFX;
import frc.robot.Subsystems.Intake.RollerSim;
import frc.robot.Subsystems.Intake.RollerTalonFX;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants;

public class IntakeFactory {
    public static IntakeConstants kIntakeConstants = RobotConstants.IntakeConstants();

    public static Intake create(SubsystemType type) {
        switch (type) {
            case REAL:
                return new Intake(new RollerTalonFX(), new PivotTalonFX());
            case SIM:
                return new Intake(new RollerSim(), new PivotSim());
            default:
                return Intake.NoOp;
        }
    }
}
