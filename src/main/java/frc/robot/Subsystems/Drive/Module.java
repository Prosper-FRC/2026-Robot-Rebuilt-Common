package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {
    private final ModuleIO kModule;
    private final moduleInputsAutoLogged kInputs = new moduleInputsAutoLogged();
    private final String kName;
    private final double kCANCoderOffset;

    public Module(ModuleIO module, String name, double CANCoderOffset) {
        kModule = module;
        kName = name;
        kCANCoderOffset = CANCoderOffset;
    }

    // Drive methods
    public void setDriveRotations(double rotations) {
        kModule.setDriveRotations(rotations);
    }

    public void setDriveMPS(double mps) {
        kModule.setDriveRPS(mps/DriveConstants.getInstance().kHardwareSpecifictions.kWheelRadiusMeters() / (2 * (Math.PI)));
    }
    
    public void setDriveVoltage(double volts) {
        double limit = Math.abs(DriveConstants.getInstance().kMaxVoltage);
        kModule.setDriveVoltage(MathUtil.clamp(volts, -limit, limit));
    }

    public void stopDrive() {
        kModule.stopDrive();
    }

    public void resetDrive() {
        kModule.resetDrive();
    }

    // Azimuth methods
    public void setAzimuthRotations(double rotations) {
        kModule.setAzimuthRotations(rotations);
    }

    public void setAzimuthRPS(double rps) {
        kModule.setAzimuthRPS(rps);
    }

    public void setAzimuthVoltage(double volts) {
        double limit = Math.abs(DriveConstants.getInstance().kMaxVoltage);
        kModule.setAzimuthVoltage(MathUtil.clamp(volts, -limit, limit));
    }

    public void stopAzimuth() {
        kModule.stopAzimuth();
    }

    public void resetAzimuth() {
        kModule.resetAzimuth(kCANCoderOffset);
    }
    
    // Misc (No implementation just yet)
    public void updateDrivePIDValues(double kP, double kI, double kD) {}

    public void updateAzimuthPIDValues(double kP, double kI, double kD) {}

    public SwerveModulePosition getAsSwerveModulePosition() {
        return new SwerveModulePosition(Distance.ofBaseUnits(rotationsToMeters(kInputs.drivePositionRotations), Meters), new Rotation2d(kInputs.azimuthPositionRotations * (2 * Math.PI)));
    }

    // Helpers
    public double rotationsToMeters(double rotations) {
        return rotations * (2 * Math.PI) * DriveConstants.getInstance().kHardwareSpecifictions.kWheelRadiusMeters();
    }

    @Override
    public void periodic() {
        kModule.updateInputs(kInputs);
        Logger.processInputs("Modules/" + kName, kInputs);
    }
}
