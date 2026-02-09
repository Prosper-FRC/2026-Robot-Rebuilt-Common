package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants.gyroOffsets;

public class GyroPigeon2 implements GyroIO {
    // Create an object to store the gyro
    private final Pigeon2 kGyro;

    // Create an instance the holds the gyro configurations.
    private final Pigeon2Configuration kGyroConfiguration = new Pigeon2Configuration();

    // Create objects to store the status signals of the Gyro.
    private final StatusSignal<Angle> kRoll;
    private final StatusSignal<Angle> kPitch;
    private final StatusSignal<Angle> kYaw;

    public GyroPigeon2(int GyroID, gyroOffsets offsets, String CANBus) {
        kGyro = new Pigeon2(GyroID, CANBus);

        // Apply offsets the the gyroscope
        kGyroConfiguration.MountPose.MountPoseRoll = offsets.roll();
        kGyroConfiguration.MountPose.MountPosePitch = offsets.pitch();
        kGyroConfiguration.MountPose.MountPoseYaw = offsets.yaw();

        // Store the references to the gyro sensor readings.
        kRoll = kGyro.getRoll();
        kPitch = kGyro.getPitch();
        kYaw = kGyro.getYaw();
    }

    @Override
    public void updateInputs(gyroInputs toUpdate) {
        toUpdate.isOk = BaseStatusSignal.refreshAll(
            kRoll, kPitch, kYaw
        ).isOK();

        toUpdate.rollRotations = kRoll.getValueAsDouble()/360;
        toUpdate.pitchRotations = kPitch.getValueAsDouble()/360;
        toUpdate.yawRotations = kYaw.getValueAsDouble()/360;
    }

    @Override
    public void updateGyro(double yaw) {
        kGyro.setYaw(kYaw.getValueAsDouble() + yaw);
    }

    @Override 
    public void setGyro(double yaw) {
        kGyro.setYaw(yaw);
    }

    @Override
    public void resetGyro() {
        kGyro.setYaw(0.0d);
    }
}
