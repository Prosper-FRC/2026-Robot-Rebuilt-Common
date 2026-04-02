package frc.robot.Factories;

import frc.robot.Factories.SubsystemFactory.SubsystemType;
import frc.robot.Subsystems.Shooter.FlywheelSim;
import frc.robot.Subsystems.Shooter.FlywheelTalonFX;
import frc.robot.Subsystems.Shooter.HoodRev;
import frc.robot.Subsystems.Shooter.HoodSim;
import frc.robot.Subsystems.Shooter.Shooter;

public class ShooterFactory {
    public static Shooter create(SubsystemType type) {
        switch(type) {
            case REAL:
                return new Shooter(
                    new FlywheelTalonFX(),
                    new HoodRev()
                );
            case SIM:
                return new Shooter(
                    new FlywheelSim(), 
                    new HoodSim()
                );
            default:
                return Shooter.NoOp;
        }
    }
}
