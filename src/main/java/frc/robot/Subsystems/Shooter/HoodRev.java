package frc.robot.Subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.RobotConstants;
import frc.robot.Subsystems.Shooter.ShooterConstants.ShooterConstants;

public class HoodRev implements HoodIO {
    private final SparkMax kHoodMotor;
    private final SparkMaxConfig kHoodConfig;

    private final ShooterConstants kHConsts = RobotConstants.ShooterConstants();

    public HoodRev() {
        kHoodMotor = new SparkMax(kHConsts.hoodID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        kHoodConfig = new SparkMaxConfig();
        var gains = kHConsts.kHGains;
        kHoodConfig.closedLoop.p(gains.kP());
        kHoodConfig.closedLoop.i(gains.kI());
        kHoodConfig.closedLoop.d(gains.kD());
        kHoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        kHoodMotor.configure(kHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    
    @Override
    public void updateInputs(hoodInputs toUpdate) {
        toUpdate.isOk = true;
        toUpdate.hoodPositionRotation = kHoodMotor.getEncoder().getPosition();
        toUpdate.hoodVelocityRPS = kHoodMotor.getEncoder().getVelocity();
        toUpdate.voltage = kHoodMotor.getBusVoltage();
        toUpdate.supplyCurrent = kHoodMotor.getOutputCurrent();
    }

    @Override
    public void setHoodPosition(double rotations) {
        kHoodMotor.getClosedLoopController().setSetpoint(rotations, ControlType.kPosition);
    }

    @Override
    public void setHoodVoltage(double volts) {
        kHoodMotor.setVoltage(volts);
    }

    @Override
    public void stopHood() {
        kHoodMotor.stopMotor();
    }
    
}
