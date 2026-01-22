package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
    private final GyroIO kGyro;
    private final gyroInputsAutoLogged kInputs = new gyroInputsAutoLogged();

    public Gyro(GyroIO gyro) {
        kGyro = gyro;
    }

    public Rotation3d getRotations() {
        return new Rotation3d(kInputs.rotationsRoll, kInputs.rotationsPitch, kInputs.rotationsYaw);
    }

    public Rotation3d getRPS() {
        return new Rotation3d(kInputs.rpsRoll, kInputs.rpsPitch, kInputs.rpsYaw);
    }

    @Override
    public void periodic() {
        kGyro.updateInputs(kInputs);
    }
}
