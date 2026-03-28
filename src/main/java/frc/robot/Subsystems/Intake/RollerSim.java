package frc.robot.Subsystems.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants.IntakeGains;

public class RollerSim implements RollerIO { 
    private double appliedVoltage = 0.0d;
    private boolean usePID = false;
    private double targetSpeedSetpoint = 0.0d;
    private final PIDController kRollerController;
    private final SimpleMotorFeedforward kRollerFeedforward;

    private final DCMotorSim kRollerMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.05, RobotConstants.IntakeConstants().kIntakeHardLimits.rollerGearRatio()), 
        DCMotor.getKrakenX60(1), 
        0.0d, 0.0d);

    public RollerSim() {
        IntakeGains gains = RobotConstants.IntakeConstants().kIntakeRollerGains;
        kRollerController = new PIDController(gains.pidGains().kP(), gains.pidGains().kI(), gains.pidGains().kD());
        kRollerFeedforward = new SimpleMotorFeedforward(gains.feedForwardGains().kS(), gains.feedForwardGains().kV());
    }

    @Override
    public void updateInputs(RollerInputs toUpdate) {
        toUpdate.isOk = true;
        toUpdate.positionRotations = kRollerMotor.getAngularPositionRotations();
        toUpdate.velocityRPS = kRollerMotor.getAngularVelocityRPM() / 60;
        toUpdate.supplyVoltage = appliedVoltage;

        if(usePID) {
            appliedVoltage = kRollerController.calculate(appliedVoltage, targetSpeedSetpoint) + kRollerFeedforward.calculate(kRollerMotor.getAngularVelocityRPM() / 60);
        }
        kRollerMotor.setInputVoltage(appliedVoltage);

        kRollerMotor.update(RobotConstants.Instance().kTimestep);
    }

    @Override
    public void setOutputVoltage(double voltage) {
        appliedVoltage = voltage;
        usePID = false;
    }

    @Override
    public void setTargetVelocity(Rotation2d velocityPerSecond) {
        usePID = true;
        targetSpeedSetpoint = velocityPerSecond.getRotations();
    }

    @Override
    public void stopRollerMotor() {
        usePID = false;
        appliedVoltage = 0.0d;
    }
}
