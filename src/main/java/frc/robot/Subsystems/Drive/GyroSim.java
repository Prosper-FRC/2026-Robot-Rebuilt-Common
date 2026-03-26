package frc.robot.Subsystems.Drive;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

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
    
    @Override
    public Optional<Rotation2d> getGyroAngle() {
        return Optional.ofNullable(Rotation2d.fromRotations(yaw));
    }
}
