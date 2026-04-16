package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;

public class PivotSim implements PivotIO {
    @AutoLogOutput(key = "Intake/Pivot/Voltage")
    private double appliedVoltage = 0.0d;
    @AutoLogOutput(key = "Intake/Pivot/UsePID")
    private boolean usePID = false;
    @AutoLogOutput(key = "Intake/Pivot/TargetPose")
    private double targetPosition = 0.0d;
    private final PIDController kPivotController;

    private final SingleJointedArmSim kPivotMotor = new SingleJointedArmSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.0125d, RobotConstants.IntakeConstants().kIntakeHardLimits.pivotGearRatio()),
        DCMotor.getKrakenX60(1),
        RobotConstants.IntakeConstants().kIntakeHardLimits.pivotGearRatio(),
        0.65d, 
        Units.rotationsToRadians(-2.0d), 
        Units.rotationsToRadians(2.0d),
        true, 
        0.0d,
        appliedVoltage, 
        0.0d);

    public PivotSim() {
        kPivotController = RobotConstants.IntakeConstants().kSimIntakePID;
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.isOk = true;
        inputs.positionRotations = Units.radiansToRotations(kPivotMotor.getAngleRads());
        inputs.velocityRPS = Units.radiansToRotations(kPivotMotor.getVelocityRadPerSec());
        inputs.supplyVoltage = appliedVoltage;

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
