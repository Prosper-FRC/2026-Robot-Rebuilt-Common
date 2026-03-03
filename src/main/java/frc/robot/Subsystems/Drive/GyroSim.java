package frc.robot.Subsystems.Drive;

public class GyroSim implements GyroIO {

    public GyroSim() {}

    private double yaw = 0.0d;

    @Override
    public void updateInputs(gyroInputs toUpdate) {
        toUpdate.isOk = true;
        toUpdate.pitchRotations = 0.0d;
        toUpdate.rollRotations = 0.0d;
        toUpdate.yawRotations = yaw;
    }

    @Override
    public void updateGyro(double yaw) {
        this.yaw += yaw;
    }

}
