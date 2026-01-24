package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Subsystems.Drive.GyroIO.gyroInputs;

public class GyroSim implements GyroIO {
    private static double yaw = 0.0d;
    private static double deltaYaw = 0.0d;

    public GyroSim() {}

    @Override
    public void updateInputs(gyroInputs toUpdate) {
        toUpdate.rotationsYaw = yaw;
        toUpdate.rotationsRoll = 0.0d;
        toUpdate.rotationsPitch = 0.0d;
        toUpdate.rpsYaw = deltaYaw;
        toUpdate.rpsRoll = 0.0d;
        toUpdate.rpsPitch = 0.0d;
    }

    @Override
    public double getYawAngleRotations() {
        return yaw;
    }

    @Override
    public double getYawAngleRPS() {
        return deltaYaw;
    }

    @Override
    public void updateYaw(double omegaSecond, double dt) {
        deltaYaw = omegaSecond;
        yaw += omegaSecond * dt;
    }
}
