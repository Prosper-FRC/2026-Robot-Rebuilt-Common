package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;

public class ModuleSim implements ModuleIO {
    private final DCMotorSim kDrive;
    private final DCMotorSim kAzimuth;
    
    private final PIDController kDriveController;
    private final PIDController kAzimuthController;
    private final SimpleMotorFeedforward kDriveFeedforward;

    @AutoLogOutput(key = "Drive/Sim/DriveGoal")
    private double driveGoal = 0.0d;
    @AutoLogOutput(key = "Drive/Sim/AzimuthGoal")
    private double azimuthGoal = 0.0d;
    @AutoLogOutput(key = "Drive/Sim/DriveVoltage")
    private double driveAppliedVoltage = 0.0d;
    @AutoLogOutput(key = "Drive/Sim/AzimuthVoltage")
    private double azimuthAppliedVoltage = 0.0d;

    public ModuleSim() {
        // Set up motor simulators
        kDrive = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.004d, RobotConstants.DriveConstants().kModuleHardLimits.driveGearRatio()), DCMotor.getKrakenX60(1), 0.0d, 0.0d);
        kAzimuth = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.025d, RobotConstants.DriveConstants().kModuleHardLimits.azimuthGearRatio()), DCMotor.getKrakenX44(1), 0.0d, 0.0d);

        // Get PID values
        kDriveController = RobotConstants.DriveConstants().kSimDrivePID;
        kAzimuthController = RobotConstants.DriveConstants().kSimAzimuthPID;
        kDriveFeedforward = RobotConstants.DriveConstants().kSimDriveFeedforward;
    }

    @Override
    public void updateInputs(moduleInputs toUpdate) {
        // Update inputs
        toUpdate.driveOk = true;
        toUpdate.azimuthOk = true;
        toUpdate.CANCoderOk = true;

        toUpdate.drivePositionRotations = kDrive.getAngularPositionRotations();
        toUpdate.driveVelocityRPS = kDrive.getAngularVelocityRPM()/60;
        toUpdate.driveSupplyCurrent = kDrive.getCurrentDrawAmps();
        toUpdate.driveSupplyVoltage = kDrive.getInputVoltage();

        toUpdate.azimuthPositionRotations = kAzimuth.getAngularPositionRotations();
        toUpdate.azimuthVelocityRPS = kAzimuth.getAngularVelocityRPM()/60;
        toUpdate.azimuthSupplyCurrent = kAzimuth.getCurrentDrawAmps();
        toUpdate.azimuthSupplyVoltage = kAzimuth.getInputVoltage();

        // Update PID
        kAzimuthController.enableContinuousInput(-0.5d, 0.5d);

        driveAppliedVoltage = kDriveController.calculate(toUpdate.driveVelocityRPS, driveGoal) + kDriveFeedforward.calculate(driveGoal);
        azimuthAppliedVoltage = kAzimuthController.calculate(toUpdate.azimuthPositionRotations, azimuthGoal);
        
        driveAppliedVoltage = MathUtil.clamp(driveAppliedVoltage, -12.0d, 12.0d);
        azimuthAppliedVoltage = MathUtil.clamp(azimuthAppliedVoltage, -12.0d, 12.0d);

        kDrive.setInputVoltage(driveAppliedVoltage);
        kAzimuth.setInputVoltage(azimuthAppliedVoltage);

        kDrive.update(RobotConstants.Instance().kTimestep);
        kAzimuth.update(RobotConstants.Instance().kTimestep);
    }

    // Drive methods
    @Override
    public void setDriveRPS(double rps) {
        driveGoal = rps;
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVoltage = volts;
    }

    @Override
    public void stopDrive() {
        driveGoal = 0.0d;
        driveAppliedVoltage = 0.0d;
    }

    @Override
    public void resetDrive() {
        kDrive.setAngle(0.0d);
    }

    // Azimuth methods
    @Override
    public void setAzimuthRotations(double rotations) {
        azimuthGoal = rotations;
    }

    @Override
    public void setAzimuthVoltage(double volts) {
        azimuthAppliedVoltage = volts;
    }

    @Override
    public void stopAzimuth() {
        azimuthGoal = 0.0d;
        azimuthAppliedVoltage = 0.0d;
    }
} 
