package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Subsystems.Shooter.ShooterConstants.FlywheelGains;
import frc.robot.Subsystems.Shooter.ShooterConstants;

public class HooderIOSim implements HooderIO {
    private static final double kLoopPeriodSec = 0.02;

    private final SingleJointedArmSim kHooder;

    @AutoLogOutput(key = "Shooter/HooderPositionRadiansGoal")
    private double kHooderPositionRadiansGoal = 0.0;

    // Hooder feedforward and feedback declarations
    private final ProfiledPIDController hooderPIDController;
    private ArmFeedforward hooderFeedforward;
    private TrapezoidProfile hooderTrapezoidProfile;

    public HooderIOSim(ShooterConstants.HooderGains hooderGains) {
        System.out.println(DCMotor.getKrakenX44Foc(1));
        kHooder = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getKrakenX44Foc(1), 0.001, 50.0), 
            DCMotor.getNeo550(1), 
            50.0, 
            ShooterConstants.getInstance().kHooderArmLengthMeters,
            Units.degreesToRadians(0),
            Units.degreesToRadians(90),
            true,
            0.0
        );
       
        // Hooder feedforward and feedback initialization
        hooderTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(hooderGains.maxVelocityRadiansPerSecond(), hooderGains.maxAccelerationRadiansPerSecondSquared()));

        hooderPIDController = new ProfiledPIDController(
            hooderGains.p(), 
            hooderGains.i(),
            hooderGains.d(),
            new TrapezoidProfile.Constraints(hooderGains.maxVelocityRadiansPerSecond(), hooderGains.maxAccelerationRadiansPerSecondSquared())
        );

        hooderFeedforward = new ArmFeedforward(hooderGains.s(), hooderGains.g(), hooderGains.v(), hooderGains.a());
    }

    @Override
    public void setHooderVoltage(double volts) {
        kHooder.setInputVoltage(volts);
    }

    @Override
    public void setHooderPositionRotationsGoal(Rotation2d newHoodPosition) {
        kHooderPositionRadiansGoal = newHoodPosition.getRadians();
        hooderPIDController.setGoal(kHooderPositionRadiansGoal);
    }

    @Override
    public void stopHooder() {
        kHooder.setInputVoltage(0.0);
    }

    @Override
    public double getHooderPositionRadiansGoal() {
        return kHooderPositionRadiansGoal;
    }

    public double calculateHooderVolts(double goalPosition) {
        double PIDVolts = hooderPIDController.calculate(kHooder.getAngleRads(), goalPosition);
        double setpointVelocity = hooderPIDController.getSetpoint().velocity;
        double feedforwardVolts = hooderFeedforward.calculate(kHooder.getAngleRads(), setpointVelocity);

        return (PIDVolts + feedforwardVolts);
    }

    @Override
    public void updateInputs(HooderInputs toUpdate) {
        kHooder.setInputVoltage(calculateHooderVolts(kHooderPositionRadiansGoal));
        kHooder.update(kLoopPeriodSec);

        // Hooder
        toUpdate.hooderOk = true;
        toUpdate.hooderAngleRads = kHooder.getAngleRads();
        toUpdate.hooderVelocityRPM = kHooder.getVelocityRadPerSec();
        toUpdate.hooderVoltage = kHooder.getInput(0);
        toUpdate.hooderStatorCurrent = (kHooder.getInput(0) > 0.0) ? kHooder.getCurrentDrawAmps() : 0.0;
        toUpdate.hooderSupplyCurrent = (kHooder.getInput(0) > 0.0) ? kHooder.getCurrentDrawAmps() : 0.0;
    }

    @Override
    public void setHooderGains(double p, double i, double d, double s, double g, double v, double a)  {
        hooderPIDController.setPID(p, i, d);
        hooderFeedforward = new ArmFeedforward(s, g, v, a);
    }

    @Override
    public void setHooderMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
        hooderTrapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }
}
