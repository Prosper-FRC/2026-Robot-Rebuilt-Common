package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Shooter.ShooterConstants.ShooterConstants.shooterGains;

public class FlywheelSim implements FlywheelIO {
    private final shooterGains kGains = RobotConstants.ShooterConstants().kShooterGains;
    
    private final DCMotorSim kFlywheelMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.015, 1.0d), 
        DCMotor.getKrakenX60(1), 
        0.0d, 0.0d
    );

    private final PIDController kFlywheelController = new PIDController(kGains.kP(), kGains.kI(), kGains.kD());
    private final SimpleMotorFeedforward kFlywheelFeedforward = new SimpleMotorFeedforward(0.0d, kGains.kV());

    @AutoLogOutput(key = "Shooter/Flywheel/AppliedVoltage")
    private double appliedVoltage = 0.0d;
    
    @AutoLogOutput(key = "Shooter/Flywheel/TargetSetpoint")
    private double setpoint = 0.0d;

    @AutoLogOutput(key = "Shooter/Flywheel/UsePID")
    private boolean useSetpoint = false;

    public FlywheelSim() {}

    @Override
    public void updateInputs(flywheelInputs toUpdate) {
        toUpdate.isOk = true;
        toUpdate.appliedVoltage = kFlywheelMotor.getInputVoltage();
        toUpdate.velocityRPS = kFlywheelMotor.getAngularVelocityRPM() / 60;
        toUpdate.statorCurrent = -1.0d;
        toUpdate.supplyCurrent = kFlywheelMotor.getCurrentDrawAmps();

        if(useSetpoint) {
            appliedVoltage = kFlywheelController.calculate(toUpdate.velocityRPS, setpoint) + kFlywheelFeedforward.calculate(setpoint);
        }
        kFlywheelMotor.setInputVoltage(appliedVoltage);

        kFlywheelMotor.update(RobotConstants.Instance().kTimestep);
    }

    @Override
    public void setFlywheelRPS(double rps) {
        useSetpoint = true;
        setpoint = rps;
    }

    @Override
    public void setFlywheelVoltage(double voltage) {
        useSetpoint = false;
        appliedVoltage = voltage;
    }

    @Override
    public void stopFlywheel() {
        useSetpoint = false;
        kFlywheelMotor.setInputVoltage(0.0d);
    }
}
