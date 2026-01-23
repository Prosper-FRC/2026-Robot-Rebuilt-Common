package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleSim implements ModuleIO {
    private final DCMotorSim kDriveMotor;
    private final DCMotorSim kAzimuthMotor;
    private final PIDController kDriveController;
    private final PIDController kAzimuthController;
    private double driveTargetValue = 0.0d;
    private double appliedDriveVoltage = 0.0d;
    private double azimuthTargetValue = 0.0d;
    private double appliedAzimtuhVoltage = 0.0d;

    public ModuleSim() {
        kDriveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025d, DriveConstants.getInstance().kDriveGearing), DCMotor.getKrakenX60Foc(1), 0.0d, 0.0d);
        kAzimuthMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025d, DriveConstants.getInstance().kAzimuthGearing), DCMotor.getKrakenX60Foc(1), 0.0d, 0.0d);

        kDriveController = DriveConstants.getInstance().kDrivePIDController;
        kAzimuthController = DriveConstants.getInstance().kAzimuthPIDController;
    }

    @Override
    public void updateInputs(moduleInputs toUpdate) {
        toUpdate.driveOk = true;
        toUpdate.azimuthOk = true;

        toUpdate.azimuthPositionRotations = kAzimuthMotor.getAngularPositionRotations();
        toUpdate.azimuthVelocityRPS = kAzimuthMotor.getAngularVelocityRPM() / 60;
        toUpdate.azimuthSupplyCurrent = kAzimuthMotor.getCurrentDrawAmps();
        toUpdate.azimuthStatorCurrent = kAzimuthMotor.getCurrentDrawAmps();
        toUpdate.azimuthSupplyVoltage = kAzimuthMotor.getInputVoltage();
        toUpdate.azimuthTemperatureCelcius = 20;

        toUpdate.azimuthPositionRotations = kDriveMotor.getAngularPositionRotations();
        toUpdate.azimuthVelocityRPS = kDriveMotor.getAngularVelocityRPM() / 60;
        toUpdate.azimuthSupplyCurrent = kDriveMotor.getCurrentDrawAmps();
        toUpdate.azimuthStatorCurrent = kDriveMotor.getCurrentDrawAmps();
        toUpdate.azimuthSupplyVoltage = kDriveMotor.getInputVoltage();
        toUpdate.azimuthTemperatureCelcius = 20;

        appliedDriveVoltage = kDriveController.calculate(toUpdate.driveVelocityRPS, driveTargetValue);
        appliedAzimtuhVoltage = kAzimuthController.calculate(toUpdate.azimuthPositionRotations, azimuthTargetValue);
        kDriveMotor.setInputVoltage(appliedDriveVoltage);
        kAzimuthMotor.setInputVoltage(appliedAzimtuhVoltage);

        kDriveMotor.update(0.02d);
        kAzimuthMotor.update(0.02d);
    }

    // Drive methods
    @Override
    public void setDriveRPS(double rps) {
        driveTargetValue = rps;
    }

    @Override
    public void setDriveVoltage(double volts) {
        appliedDriveVoltage = volts;
        driveTargetValue = 0.0d;
    }

    @Override
    public void stopDrive() {
        appliedDriveVoltage = 0.0d;
        driveTargetValue = 0.0d;
    }

    @Override
    public void resetDrive() {
        kDriveMotor.setAngle(0.0);
    }

    // Azimuth methods
    @Override
    public void setAzimuthRotations(double rotations) {
        azimuthTargetValue = rotations;
    }

    @Override
    public void setAzimuthVoltage(double volts) {
        azimuthTargetValue = 0.0d;
        appliedAzimtuhVoltage = volts;
    }

    @Override
    public void stopAzimuth() {
        azimuthTargetValue = 0.0d;
        appliedAzimtuhVoltage = 0.0d;
    }

    @Override
    public void resetAzimuth(double position) {
        kAzimuthMotor.setAngle(0.0d);
    }

} 
