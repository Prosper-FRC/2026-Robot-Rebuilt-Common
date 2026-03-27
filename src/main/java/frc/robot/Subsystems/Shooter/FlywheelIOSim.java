package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Shooter.ShooterConstants.ShooterConstants.FlywheelGains;

public class FlywheelIOSim implements FlywheelIO {
    private static final double kLoopPeriodSec = 0.02;

    private final FlywheelSim kFlywheel;

    @AutoLogOutput(key = "Shooter/FlywheelVelocityGoalRPS")
    private double kFlywheelVelocityGoal = 0.0;

    // Flywheel feedforward and feedback declarations
    private final PIDController flywheelPIDController;
    private SimpleMotorFeedforward flywheelFeedforward;
    private SlewRateLimiter flywhweelAccelerationLimiter;

    public FlywheelIOSim(FlywheelGains flywheelGains) {
        kFlywheel = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44Foc(1), 0.001, 0.1),
            DCMotor.getKrakenX44Foc(1)
        );
        System.out.println(DCMotor.getKrakenX44Foc(1));

        // Flywheel feedforward and feedback initialization
        flywheelPIDController = new PIDController(
            flywheelGains.p(), 
            flywheelGains.i(),
            flywheelGains.d()
        );

        flywheelFeedforward = new SimpleMotorFeedforward(flywheelGains.s(), flywheelGains.v(), flywheelGains.a());
        flywhweelAccelerationLimiter = new SlewRateLimiter(flywheelGains.maxAccelerationRadiansPerSecondSquared());
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        kFlywheel.setInputVoltage(volts);
    }

    @Override
    public void setFlywheelVelocity(double velocity) {
        kFlywheelVelocityGoal = velocity;
    }

    @Override
    public void stopFlywheel() {
        setFlywheelVelocity(RobotConstants.ShooterConstants().kFlywheelVelocityOffRadiansPerSec);
    }

    public double calculateFlywheelVolts(double goalVelocity) {
        double PIDVolts = flywheelPIDController.calculate(kFlywheel.getAngularVelocityRadPerSec(), goalVelocity);
        double feedforwardVolts = flywheelFeedforward.calculate(goalVelocity);

        return (PIDVolts + feedforwardVolts);
    }

    @Override
    public void updateInputs(FlywheelInputs toUpdate) {
        kFlywheel.setInputVoltage(calculateFlywheelVolts(flywhweelAccelerationLimiter.calculate(kFlywheelVelocityGoal)));

        kFlywheel.update(kLoopPeriodSec);

        // Flywheel
        toUpdate.flywheelOk = true;
        toUpdate.flywheelVoltage = (kFlywheel.getInputVoltage() > 0.0) ? 12.0 : 0.0;
        toUpdate.flywheelVelocityRPM = kFlywheel.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        toUpdate.flywheelStatorCurrent = (kFlywheel.getInputVoltage() > 0.0) ? kFlywheel.getCurrentDrawAmps() : 0.0;
        toUpdate.flywheelSupplyCurrent = (kFlywheel.getInputVoltage() > 0.0) ? kFlywheel.getCurrentDrawAmps() : 0.0;
    }
}
