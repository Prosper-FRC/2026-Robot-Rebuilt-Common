package frc.robot.Subsystems.Drive.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * IMPORTANT NOTE: Not all of the information here is currently used in the setpoint generator, but it might be in the future. Ensure you are properly setting all relevant parameters.
 */
public class SwerveConfiguration {
    public static final record ChassisTranslations(
        Translation2d frontLeftTranslation,
        Translation2d frontRightTranslation,
        Translation2d backLeftTranslation,
        Translation2d backRightTranslation
    ) {}

    public static final record ModuleConstraints(
        double maximumLinearVelocityMPS,
        double maximumAngularVelocityRPS
    ) {}

    public static final record SwerveHardware(
        double wheelRadiusMeters,
        double chassisSideLengthMeters
    ) {}

    public SwerveConfiguration() {}

    public ChassisTranslations chassisTranslations = new ChassisTranslations(
        new Translation2d(-0.5d, 0.5d), 
        new Translation2d(0.5d, 0.5d),
        new Translation2d(-0.5d, -0.5d), 
        new Translation2d(0.5d, -0.5d)
    );

    public ModuleConstraints moduleConstraints = new ModuleConstraints(
        4.5d,
        1.0d 
    );

    public SwerveHardware swerveHardware = new SwerveHardware(
        Units.inchesToMeters(2.0d),
        Units.inchesToMeters(36.0d)
    );

    public SwerveConfiguration withChassisTranslations(ChassisTranslations translations) {
        chassisTranslations = translations;
        return this;
    }

    public SwerveConfiguration withModuleConstraints(ModuleConstraints constraints) {
        moduleConstraints = constraints;
        return this;
    }

    public SwerveConfiguration withSwerveHardware(SwerveHardware hardware) {
        swerveHardware = hardware;
        return this;
    }
}
