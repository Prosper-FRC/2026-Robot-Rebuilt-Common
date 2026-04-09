package frc.robot.Subsystems.Drive.SwerveSetpointGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

// NOTE: Most things is this folder are hardcoded due to the fact that this folder is designed to move across codebases.
public class SetpointGenerator {
    private final SwerveConfiguration setpointGeneratorConfiguration;
    private final SwerveDriveKinematics swerveKinematics;

    public SetpointGenerator(SwerveConfiguration configuration) {    
        if(configuration == null) {
            throw new NullPointerException("SwerveConfiguration cannot be null");
        }

        setpointGeneratorConfiguration = configuration;

        swerveKinematics = new SwerveDriveKinematics(
          setpointGeneratorConfiguration.chassisTranslations.frontLeftTranslation(),
          setpointGeneratorConfiguration.chassisTranslations.frontRightTranslation(),
          setpointGeneratorConfiguration.chassisTranslations.backLeftTranslation(),
          setpointGeneratorConfiguration.chassisTranslations.backRightTranslation()  
        );
    }

    /**
     * Takes in desired speeds and current states, and spits out new target speeds, expects 4 modules in the order of FL, FR, BL, BR.
     * @param desiredSpeeds The desired speeds for the robot in the robot's frame of reference.
     * @param currentModuleStates The currently read states of the module in the robot's frame of reference.
     * @return The new target speeds of the modules.
     */
    public SwerveModuleState[] generateSetpoint(ChassisSpeeds desiredSpeeds, SwerveModuleState[] currentModuleStates) {
        if(currentModuleStates.length != 4) {
            throw new IllegalArgumentException("Invalid currentModuleStates size, expected size 4, got size " + String.valueOf(currentModuleStates.length));
        }

        ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 0.02d);
        double angularVelocity = discretizedSpeeds.omegaRadiansPerSecond;
        double cappedAngularVelocity = MathUtil.clamp(
            angularVelocity, 
            -Units.rotationsToRadians(setpointGeneratorConfiguration.moduleConstraints.maximumAngularVelocityRPS()), 
            Units.rotationsToRadians(setpointGeneratorConfiguration.moduleConstraints.maximumAngularVelocityRPS())
        );
        discretizedSpeeds.omegaRadiansPerSecond = cappedAngularVelocity;
        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(discretizedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, setpointGeneratorConfiguration.moduleConstraints.maximumLinearVelocityMPS());
        optimizeAll(moduleStates, currentModuleStates);
    
        return moduleStates;
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveKinematics;
    }

    private void optimizeAll(SwerveModuleState[] modules, SwerveModuleState[] currentModules) {
        for(int i = 0; i < 4; ++i) {
            modules[i].optimize(currentModules[i].angle);
        }
    }
}