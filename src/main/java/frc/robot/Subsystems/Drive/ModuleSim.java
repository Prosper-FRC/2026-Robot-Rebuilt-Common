package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleSim implements ModuleIO {
    private final DCMotorSim kDriveMotor;
    private final DCMotorSim kAzimuthMotor;
    private final PIDController kDriveController;
    private final PIDController kAzimuthController;
    @AutoLogOutput(key = "Drive/TargetDrive")
    private double driveTargetValue = 0.0d;
    @AutoLogOutput(key = "Drive/VoltageDrive")
    private double appliedDriveVoltage = 0.0d;
    @AutoLogOutput(key = "Drive/TargetAzimuth")
    private double azimuthTargetValue = 0.0d;
    @AutoLogOutput(key = "Drive/VoltageAzimuth")
    private double appliedAzimtuhVoltage = 0.0d;

    public ModuleSim() {
        kDriveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025d, DriveConstants.getInstance().kDriveGearing), DCMotor.getKrakenX60Foc(1), 0.0d, 0.0d);
        kAzimuthMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025d, DriveConstants.getInstance().kAzimuthGearing), DCMotor.getKrakenX60Foc(1), 0.0d, 0.0d);

        kDriveController = DriveConstants.getInstance().kDrivePIDController;
        kAzimuthController = DriveConstants.getInstance().kAzimuthPIDController;
        kAzimuthController.enableContinuousInput(-0.5d, 0.5d);
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

        toUpdate.drivePositionRotations = kDriveMotor.getAngularPositionRotations();
        toUpdate.driveVelocityRPS = kDriveMotor.getAngularVelocityRPM() / 60;
        toUpdate.driveSupplyCurrent = kDriveMotor.getCurrentDrawAmps();
        toUpdate.driveStatorCurrent = kDriveMotor.getCurrentDrawAmps();
        toUpdate.driveSupplyVoltage = kDriveMotor.getInputVoltage();
        toUpdate.driveTemperatureCelcius = 20;

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
