package frc.robot.Subsystems.Drive.Controllers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class TeleopController {
    @AutoLogOutput(key = "Drive/ControllerInputs/X")
    private DoubleSupplier xInput = () -> 0.0d;
    @AutoLogOutput(key = "Drive/ControllerInputs/Y")
    private DoubleSupplier yInput = () -> 0.0d;
    @AutoLogOutput(key = "Drive/ControllerInputs/Angle")
    private DoubleSupplier angleInput = () -> 0.0d;

    // Motion magic should limit the acceleration pretty well, but this is an extra soft limit to ease controller inputs for extra smoothing.
    private final SlewRateLimiter kXSlewRateLimiter = new SlewRateLimiter(RobotConstants.DriveConstants().kModuleSoftLimits.controllerLimits().controllerInputRateLimiter());
    private final SlewRateLimiter kYSlewRateLimiter = new SlewRateLimiter(RobotConstants.DriveConstants().kModuleSoftLimits.controllerLimits().controllerInputRateLimiter());
    private final SlewRateLimiter kAngleSlewRateLimiter = new SlewRateLimiter(RobotConstants.DriveConstants().kModuleSoftLimits.controllerLimits().controllerInputRateLimiter());

    public TeleopController() {}

    public void supplyControllerInputs(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angle) {
        // These should be swapped.
        xInput = y;
        yInput = x;
        angleInput = angle;
    }

    // Unlike previous years, we arent processing the chassis speeds here since the purpose of the teleop controller is only to process
    public ChassisSpeeds getDesiredSpeeds(boolean isSniperMode) {
        // Store constants in temporary variables.
        double deadband = RobotConstants.DriveConstants().kModuleSoftLimits.controllerLimits().controllerDeadband();
        double exponent = (double)RobotConstants.DriveConstants().kModuleSoftLimits.controllerLimits().controllerInputExponent();
        double maxMPS = RobotConstants.DriveConstants().kModuleSoftLimits.maxLinearVelocityMPS();
        double maxRPS = RobotConstants.DriveConstants().kModuleSoftLimits.maxAngularVelocityRPS();
        double sniperScalar = RobotConstants.DriveConstants().sniperModeScalar;

        // Read the current input states supplied to us.
        double readXInput = xInput.getAsDouble();
        double readYInput = yInput.getAsDouble();
        double readAngleInput = angleInput.getAsDouble();

        if(RobotConstants.Instance().kIsBlueAlliance) {
            readXInput *= -1;
            readYInput *= -1;            
        }

        // Apply a deadband to the controller inputs.
        double dbXInput = MathUtil.applyDeadband(readXInput, deadband);
        double dbYInput = MathUtil.applyDeadband(readYInput, deadband);
        double dbAngleInput = MathUtil.applyDeadband(readAngleInput, deadband);

        // Apply the slew rate limiter to the controller inputs.
        double rateLimitedX = kXSlewRateLimiter.calculate(dbXInput);
        double rateLimitedY = kYSlewRateLimiter.calculate(dbYInput);
        double rateLimitedAngle = kAngleSlewRateLimiter.calculate(dbAngleInput);

        // Exponentiate the inputs with a nice brand new util.
        double exponentiatedXInput = MathUtil.copyDirectionPow(rateLimitedX, exponent);
        double exponentiatedYInput = MathUtil.copyDirectionPow(rateLimitedY, exponent);
        double exponentiatedAngleInput = MathUtil.copyDirectionPow(rateLimitedAngle, exponent);

        // Convert proper controller inputs into desired speeds.
        double speedX = exponentiatedXInput * maxMPS;
        double speedY = exponentiatedYInput * maxMPS;
        double speedOmega = -Units.rotationsToRadians(exponentiatedAngleInput * maxRPS);

        // Apply an extra sniper mode scalar.
        if(isSniperMode) {
            speedX *= sniperScalar;
            speedY *= sniperScalar;
            speedOmega *= sniperScalar;
        }

        // Create initial chassis speeds so we can return the 3 inputs in a clean way.
        ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, speedOmega);
        
        return speeds;
    }
}