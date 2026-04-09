package frc.robot.Subsystems.Drive.Gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroPigeon implements GyroIO {
    private final Pigeon2 kGyro;
    private final Pigeon2Configuration kGyroConfiguration;

    private final StatusSignal<Angle> kGyroPosition;
    private final StatusSignal<AngularVelocity> kGyroVelocity;

    public GyroPigeon(int id, Rotation3d gyroOffset, CANBus canbus) {
        kGyro = new Pigeon2(id, canbus);
        kGyroConfiguration = new Pigeon2Configuration();

        kGyroConfiguration
            .withMountPose(
                new MountPoseConfigs()
                    .withMountPoseRoll(gyroOffset.getMeasureX())
                    .withMountPosePitch(gyroOffset.getMeasureY())
                    .withMountPoseYaw(gyroOffset.getMeasureZ())
            );
        
        kGyro.getConfigurator().apply(kGyroConfiguration);

        kGyroPosition = kGyro.getYaw();
        kGyroVelocity = kGyro.getAngularVelocityZWorld();

        BaseStatusSignal.setUpdateFrequencyForAll(
            250.0d,
            kGyroPosition,
            kGyroVelocity 
        );

        kGyro.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.gyroOk = BaseStatusSignal.refreshAll(
            kGyroPosition,
            kGyroVelocity  
        ).isOK();

        inputs.gyroPositionRotations = kGyroPosition.getValueAsDouble() / 360.0d;
        inputs.gyroSpeedRotationPerSeconod = kGyroVelocity.getValueAsDouble() / 360.0d;
    }

    @Override
    public void resetGyro() {
        kGyro.reset();
    }

    @Override
    public void resetGyro(double rotation) {
        kGyro.setYaw(rotation * 360.0d);
    }
}