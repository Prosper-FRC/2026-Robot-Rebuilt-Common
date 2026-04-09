package frc.robot.Subsystems.Drive.SwerveModule;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants.SwerveModuleGains;

public class SwerveModuleSim implements SwerveModuleIO {
    private final DCMotorSim kDriveMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.004d,
            RobotConstants.DriveConstants().kModuleHardware.driveGearReduction()),
            DCMotor.getKrakenX60(1).withReduction(RobotConstants.DriveConstants().kModuleHardware.driveGearReduction()), 
            0.0d, 0.0d
    );
    private final DCMotorSim kAzimuthMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 
            0.025d,
            RobotConstants.DriveConstants().kModuleHardware.azimuthGearReduction()),
            DCMotor.getKrakenX44(1).withReduction(RobotConstants.DriveConstants().kModuleHardware.azimuthGearReduction()), 
            0.0d, 0.0d
    );

    private final PIDController kDriveController;
    private final SimpleMotorFeedforward kDriveFeedforward;
    private final PIDController kAzimuthController;

    @AutoLogOutput(key = "Drive/Sim/DriveGoalRPS")
    private double driveGoalRotationsPerSecond = 0.0d;
    @AutoLogOutput(key = "Drive/Sim/AzimuthGoalRotations")
    private double azimuthGoalRotations = 0.0d;

    @AutoLogOutput(key = "Drive/Sim/UseDrivePID")
    private boolean useDrivePID = true;
    @AutoLogOutput(key = "Drive/Sim/UseAzimuthPID")
    private boolean useAzimuthPID = true;

    @AutoLogOutput(key = "Drive/Sim/DriveVoltage")
    public double driveVoltage = 0.0d;
    @AutoLogOutput(key = "Drive/Sim/AzimuthVoltage")
    public double azimuthVoltage = 0.0d;

    public SwerveModuleSim() {
        SwerveModuleGains driveGains = RobotConstants.DriveConstants().kDriveGains;
        kDriveController = RobotConstants.DriveConstants().kDriveControllerSim;
        kDriveFeedforward = RobotConstants.DriveConstants().kDriveFeedForwardSim;

        kAzimuthController = RobotConstants.DriveConstants().kAzimuthControllerSim;
        kAzimuthController.enableContinuousInput(-0.5d, 0.5d);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.driveModuleOk = true;
        inputs.driveModuleMotorVoltage = driveVoltage;
        inputs.driveModuleSupplyVoltage = kDriveMotor.getInputVoltage();
        inputs.driveModulePositionRotations = kDriveMotor.getAngularPositionRotations();
        inputs.driveModuleSpeedRotationsPerSecond = kDriveMotor.getAngularVelocityRPM() / 60;
        inputs.driveModuleAccelerationRotationsPerSecondSquared = Units.radiansToRotations(kDriveMotor.getAngularAccelerationRadPerSecSq());
        inputs.driveModuleSupplyCurrent = -1.0d;
        inputs.driveModuleStatorCurrent = -1.0d;

        inputs.azimuthModuleOk = true;
        inputs.azimuthModuleMotorVoltage = azimuthVoltage;
        inputs.azimuthModuleSupplyVoltage = kAzimuthMotor.getInputVoltage();
        inputs.azimuthModulePositionRotations = kAzimuthMotor.getAngularPositionRotations();
        inputs.azimuthModuleSpeedRotationsPerSecond = kAzimuthMotor.getAngularVelocityRPM() / 60;
        inputs.azimuthModuleAccelerationRotationsPerSecondSquared = Units.radiansToRotations(kAzimuthMotor.getAngularAccelerationRadPerSecSq());
        inputs.azimuthModuleSupplyCurrent = -1.0d;
        inputs.azimuthModuleStatorCurrent = -1.0d;

        inputs.cancoderOk = true;
        inputs.cancoderPositionRotations = inputs.azimuthModulePositionRotations;
        inputs.cancoderOffsetPositionRotations = inputs.azimuthModulePositionRotations;
        inputs.cancoderSpeedRotationsPerSecond = inputs.azimuthModuleSpeedRotationsPerSecond;

        // Update the simulation motors
        if (useDrivePID) {
            driveVoltage = kDriveController.calculate(inputs.driveModuleSpeedRotationsPerSecond, driveGoalRotationsPerSecond) + kDriveFeedforward.calculate(driveGoalRotationsPerSecond);
        }
        if (useAzimuthPID) {
            azimuthVoltage = kAzimuthController.calculate(inputs.azimuthModulePositionRotations, azimuthGoalRotations);
        }

        kDriveMotor.setInputVoltage(driveVoltage);
        kAzimuthMotor.setInputVoltage(azimuthVoltage);

        kDriveMotor.update(RobotConstants.Instance().kTimestep);
        kAzimuthMotor.update(RobotConstants.Instance().kTimestep);
    }

    @Override
    public void setDriveSpeedWithVoltage(double rotationsPerSecond) {
        useDrivePID = true;
        driveGoalRotationsPerSecond = rotationsPerSecond;
    }

    @Override
    public void setDriveSpeedWithFOC(double rotationsPerSecond) {
        useDrivePID = true;
        driveGoalRotationsPerSecond = rotationsPerSecond;
    }

    @Override
    public void setDriveMotorVoltage(double motorVoltage) {
        useDrivePID = false;
        driveVoltage = motorVoltage;
    }

    @Override
    public void stopDriveMotor() {
        useDrivePID = false;
        driveVoltage = 0.0d;

    }

    @Override
    public void setAzimuthPositionWithVoltage(double rotations) {
        useAzimuthPID = true;
        azimuthGoalRotations = rotations;
    }

    @Override
    public void setAzimuthPositionWithFOC(double rotations) {
        useAzimuthPID = true;
        azimuthGoalRotations = rotations;
    }

    @Override
    public void setAzimuthMotorVoltage(double motorVoltage) {
        useAzimuthPID = false;
        azimuthVoltage = motorVoltage;
    }

    @Override
    public void stopAzimuthMotor() {
        useAzimuthPID = false;
        azimuthVoltage = 0.0d;
    }

}
