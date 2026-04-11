package frc.robot.Subsystems.Drive.Gyro;

public class GyroSim implements GyroIO {
    private double yaw = 0.0d;

    public GyroSim() {
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.gyroOk = true;
        inputs.gyroPositionRotations = yaw;
        inputs.gyroSpeedRotationPerSeconod = (yaw - inputs.gyroSpeedRotationPerSeconod) * 0.02d;
    }

    @Override
    public void resetGyro() {
        yaw = 0.0d;
    }

    @Override
    public void resetGyro(double rotation) {
        yaw = rotation;
    }

    @Override
    public void updateGyro(double rotations) {
        yaw += rotations;
    }
}
