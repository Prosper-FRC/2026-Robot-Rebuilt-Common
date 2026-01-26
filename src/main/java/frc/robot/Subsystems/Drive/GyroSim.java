package frc.robot.Subsystems.Drive;

public class GyroSim implements GyroIO {
    private static double yaw = 0.0d;
    private static double deltaYaw = 0.0d;

    public GyroSim() {}

    @Override
    public void updateInputs(gyroInputs toUpdate) {
        toUpdate.isOK = true;
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
    public void updateYaw(double omega, double dt) {
        deltaYaw = omega;
        yaw += omega * dt;
    }

    @Override
    public void resetGyro() {
        yaw = 0.0d;
    }
}
