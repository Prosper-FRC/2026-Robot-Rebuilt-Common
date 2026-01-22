package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroPigeon2 implements GyroIO {
    private final Pigeon2 kGyro;
    private final Pigeon2Configuration kConfiguration;

    private final StatusSignal<Angle> kYaw;
    private final StatusSignal<Angle> kRoll;
    private final StatusSignal<Angle> kPitch;
    private final StatusSignal<AngularVelocity> kDeltaYaw;
    private final StatusSignal<AngularVelocity> kDeltaRoll;
    private final StatusSignal<AngularVelocity> kDeltaPitch;

    public GyroPigeon2(int gyroID) {
        kGyro = new Pigeon2(gyroID);
        kConfiguration = new Pigeon2Configuration();

        kYaw = kGyro.getYaw();
        kRoll = kGyro.getRoll();
        kPitch = kGyro.getPitch();
        kDeltaYaw = kGyro.getAngularVelocityZDevice();
        kDeltaRoll = kGyro.getAngularVelocityXDevice();
        kDeltaPitch = kGyro.getAngularVelocityYDevice();
    }

    @Override
    public void updateInputs(gyroInputs toUpdate) {
        toUpdate.isOK = BaseStatusSignal.refreshAll(kYaw, kRoll, kPitch,
        kDeltaYaw, kDeltaRoll, kDeltaPitch).isOK();

        toUpdate.rotationsYaw = kYaw.getValueAsDouble() / 360;
        toUpdate.rotationsRoll = kRoll.getValueAsDouble() / 360;
        toUpdate.rotationsPitch = kPitch.getValueAsDouble() / 360;
        toUpdate.rpsYaw = kDeltaYaw.getValueAsDouble() / 360;
        toUpdate.rpsRoll = kDeltaRoll.getValueAsDouble() / 360;
        toUpdate.rpsPitch = kDeltaPitch.getValueAsDouble() / 360;
    }
}
