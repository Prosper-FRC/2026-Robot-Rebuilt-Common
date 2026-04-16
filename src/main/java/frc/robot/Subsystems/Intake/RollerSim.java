package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;

public class RollerSim implements RollerIO { 
    @AutoLogOutput(key = "Intake/Roller/Voltage")
    private double appliedVoltage = 0.0d;

    private final DCMotorSim kRollerMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.0005, RobotConstants.IntakeConstants().kIntakeHardLimits.rollerGearRatio()), 
        DCMotor.getKrakenX60(1), 
        0.0d, 0.0d);

    public RollerSim() {}

    @Override
    public void updateInputs(RollerInputs inputs) {
        inputs.isOk = true;
        inputs.positionRotations = kRollerMotor.getAngularPositionRotations();
        inputs.velocityRPS = kRollerMotor.getAngularVelocityRPM() / 60;
        inputs.supplyVoltage = appliedVoltage;

        kRollerMotor.setInputVoltage(appliedVoltage);

        kRollerMotor.update(RobotConstants.Instance().kTimestep);
    }

    @Override
    public void setOutputVoltage(double voltage) {
        appliedVoltage = voltage;
    }

    @Override
    public void stopRollerMotor() {
        appliedVoltage = 0.0d;
    }
}
