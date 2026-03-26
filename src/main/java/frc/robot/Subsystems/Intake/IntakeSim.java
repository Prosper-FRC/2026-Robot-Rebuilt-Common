package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeSim implements IntakeIO {
    private final SingleJointedArmSim kPivot = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 
    95.0d, SingleJointedArmSim.estimateMOI(0.3302d, 5.44311d), 0.3302, 0.0d, Units.rotationsToRadians(1.0d), true, 0.0d);

    private final DCMotorSim kRoller = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01d, 1.0d), 
    DCMotor.getKrakenX60(1));

    // PID controllers
    private final PIDController kPivotController = new PIDController(135.0d, 0.0d, 5.5d);
    private final PIDController kRollerController = new PIDController(1.75, 0.0d, 0.0d);

    private final SimpleMotorFeedforward kRollerFF = new SimpleMotorFeedforward(0.0d, 0.1175d);

    @AutoLogOutput(key = "Intake/PivotGoal")
    private double pivotGoal = 0.0d;
    @AutoLogOutput(key = "Intake/PivotVoltage")
    private double pivotVoltage = 0.0d;
    @AutoLogOutput(key = "Intake/RollerGoal")
    private double rollerGoal = 0.0d;
    @AutoLogOutput(key = "Intake/RollerVoltage")
    private double rollerVoltage = 0.0d;
    @AutoLogOutput(key = "Intake/MOIEstimation")
    private double MOI = SingleJointedArmSim.estimateMOI(0.3302d, 5.44311d); // Just a debug value.

    public IntakeSim() {}

    @Override
    public void updateInputs(IntakeInputs toUpdate) {
        // Update the inputs.
        toUpdate.isPivotOk = true;
        toUpdate.isRollerOk = true;
        toUpdate.pivotPositionRotations = Units.radiansToRotations(kPivot.getAngleRads());
        toUpdate.pivotVelocityRPS = Units.radiansToRotations(kPivot.getVelocityRadPerSec());
        toUpdate.pivotVoltage = pivotVoltage;
        toUpdate.pivotSupplyCurrent = kPivot.getCurrentDrawAmps();
        toUpdate.pivotStatorCurrent = -1.0d;
        toUpdate.pivotTemperatureCelcius = 23.0d;

        toUpdate.rollerPositionRotations = kRoller.getAngularPositionRotations();
        toUpdate.rollerVelocityRPS = kRoller.getAngularVelocityRPM() / 60;
        toUpdate.rollerVoltage = rollerVoltage;
        toUpdate.rollerSupplyCurrent = kRoller.getCurrentDrawAmps();
        toUpdate.rollerStatorCurrent = -1.0d;
        toUpdate.rollerTemperatureCelcius = 23.0d;

        pivotVoltage = kPivotController.calculate(toUpdate.pivotPositionRotations, pivotGoal);
        rollerVoltage = kRollerController.calculate(toUpdate.rollerVelocityRPS, rollerGoal) + kRollerFF.calculate(rollerGoal);

        pivotVoltage = pivotVoltage > 12.0d ? 12.0d : pivotVoltage;
        rollerVoltage = rollerVoltage > 12.0d ? 12.0d : rollerVoltage;

        kPivot.setInputVoltage(pivotVoltage);
        kRoller.setInputVoltage(rollerVoltage);

        kPivot.update(0.02d);
        kRoller.update(0.02d);
    }

    @Override
    public void setPivotPositionRotations(double position) {
        pivotGoal = position;
    }

    @Override
    public void resetPivotEncoder() {
        kPivot.setState(0.0d, kPivot.getVelocityRadPerSec());
    }

    @Override
    public void stopPivot() {
        pivotGoal = pivotVoltage;
    }

    @Override
    public void setRollerSpeedRPS(double speed) {
        rollerGoal = speed;
    }

    @Override
    public void resetRollerEncoder() {
        kRoller.setAngle(0.0d);
    }

    @Override
    public void stopRoller() {
        rollerGoal = 0.0d;
    }
}