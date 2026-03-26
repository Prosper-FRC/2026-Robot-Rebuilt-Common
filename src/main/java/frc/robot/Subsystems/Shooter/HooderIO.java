package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HooderIO {
    @AutoLog
    static class HooderInputs {
        public boolean hooderOk = false;
        public double hooderAngleRads = 0.0d;
        public double hooderVelocityRPM = 0.0d;
        public double hooderTemperatureCelcius = 0.0d;;
        public double hooderVoltage = 0.0d;
        public double hooderStatorCurrent = 0.0d;
        public double hooderSupplyCurrent = 0.0d; 
    }

    default public void updateInputs(HooderInputs toUpdate) {} 

    default public void setHooderPositionRotationsGoal(Rotation2d newHoodPosition) {}

    default public void setHooderVoltage(double volts) {}

    default public void stopHooder() {}

    default public void resetHooder() {}

    default public double getHooderPositionRadiansGoal() {return 0.0;}

    default public void setHooderGains(double p, double i, double d, double s, double g, double v, double a) {}

    default public void setHooderMotionMagicConstraints(double maxVelocity, double maxAcceleration) {}
}