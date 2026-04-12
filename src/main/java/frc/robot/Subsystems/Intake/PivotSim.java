package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Intake.IntakeConstants.IntakeConstants.IntakeGains;

public class PivotSim implements PivotIO {

    private double appliedVoltage = 0.0d;
    private boolean usePID = false;
    @AutoLogOutput(key = "Intake/TargetPose")
    private double targetPosition = 0.0d;
    private final PIDController kPivotController;

    private final SingleJointedArmSim kPivotMotor = new SingleJointedArmSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.0125d, RobotConstants.IntakeConstants().kIntakeHardLimits.rollerGearRatio()),
        DCMotor.getKrakenX60(1),
        RobotConstants.IntakeConstants().kIntakeHardLimits.rollerGearRatio(),
        0.65d, 
        0.0d, 
        RobotConstants.IntakeConstants().kIntakeSoftLimits.rangeOfMotion().getRadians(), 
        true, 
        0.0d,
        appliedVoltage, 
        0.0d);

    public PivotSim() {
        IntakeGains gains = RobotConstants.IntakeConstants().kIntakePivotGains;
        kPivotController = new PIDController(
            gains.pidGains().kP(), 
            gains.pidGains().kI(), 
            gains.pidGains().kD());
    }

    @Override
    public void updateInputs(PivotInputs toUpdate) {
        toUpdate.isOk = true;
        toUpdate.positionRotations = Units.radiansToRotations(kPivotMotor.getAngleRads());
        toUpdate.velocityRPS = Units.radiansToRotations(kPivotMotor.getVelocityRadPerSec());
        toUpdate.supplyVoltage = appliedVoltage;

        if(usePID) {
            appliedVoltage = kPivotController.calculate(Units.radiansToRotations(kPivotMotor.getAngleRads()), targetPosition); // + kPivotFeedforward.calculate(Units.radiansToRotations(kPivotMotor.getAngleRads()), Units.radiansToRotations(kPivotMotor.getVelocityRadPerSec()));
        }
        kPivotMotor.setInputVoltage(appliedVoltage);

        kPivotMotor.update(RobotConstants.Instance().kTimestep);
    }

    @Override
    public void setOutputVoltage(double voltage) {
        appliedVoltage = voltage;
        usePID = false;
    }

    @Override
    public void setTargetPosition(Rotation2d position) {
        usePID = true;
        targetPosition = position.getRotations();
    }

    @Override
    public void stopPivotMotor() {
        usePID = false;
        appliedVoltage = 0.0d;
    }
}
