package frc.robot.Factories;

import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.PivotSim;
import frc.robot.Subsystems.Intake.PivotTalonFX;
import frc.robot.Subsystems.Intake.RollerSim;
import frc.robot.Subsystems.Intake.RollerTalonFX;

public class IntakeFactory {
    private IntakeFactory() {}

    public static Intake createReal() {
        return new Intake(
            new RollerTalonFX(), 
            new PivotTalonFX()
        );
    }
    
    public static Intake createSim() {
        return new Intake(
            new RollerSim(), 
            new PivotSim()
        );
    }

    public static Intake createNoOp() {
        return Intake.NoOp;
    }
}
