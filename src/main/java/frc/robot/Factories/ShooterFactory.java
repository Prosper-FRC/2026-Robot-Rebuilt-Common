package frc.robot.Factories;

import frc.robot.Subsystems.Shooter.FlywheelSim;
import frc.robot.Subsystems.Shooter.FlywheelTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;

public class ShooterFactory {
    private ShooterFactory() {}

    public static Shooter createReal() {
        return new Shooter(
            new FlywheelTalonFX()
        );
    }
    
    public static Shooter createSim() {
        return new Shooter(
            new FlywheelSim()
        );
    }

    public static Shooter createNoOp() {
        return Shooter.NoOp;
    }
}
