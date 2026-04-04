package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;

public class HoodSim implements HoodIO {
    private final DCMotorSim kHoodMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1.0d), 
        DCMotor.getKrakenX60(1), 
        0.0d, 0.0d
    );
    private final PIDController kController;

    @AutoLogOutput(key = "Hood/SetpointPosition")
    private double setpoint = 0.0d;

    @AutoLogOutput(key = "Hood/VoltageOut")
    private double voltage = 0.0d;

    public HoodSim() {
        kController = new PIDController(
            RobotConstants.ShooterConstants().kHGains.kP(), 
            RobotConstants.ShooterConstants().kHGains.kI(), 
            RobotConstants.ShooterConstants().kHGains.kD()
        );
    }
    
    @Override
    public void updateInputs(hoodInputs toUpdate) {
        toUpdate.isOk = true;
        toUpdate.hoodPositionRotation = kHoodMotor.getAngularPositionRotations();
        toUpdate.hoodVelocityRPS = kHoodMotor.getAngularVelocityRPM() / 60;
        toUpdate.supplyCurrent = kHoodMotor.getCurrentDrawAmps();
        toUpdate.statorCurrent = -1;
        toUpdate.voltage = kHoodMotor.getInputVoltage();

        voltage = kController.calculate(toUpdate.hoodPositionRotation, setpoint);

        kHoodMotor.setInputVoltage(voltage);
        kHoodMotor.update(RobotConstants.Instance().kTimestep);
    }

    @Override
    public void setHoodPosition(double rotations) {
        setpoint = rotations;
    }

    @Override
    public void stopHood() {
        voltage = 0.0d;
    }
}
