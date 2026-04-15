// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final XboxController kController = new XboxController(0);
  private final TalonFX kShooterMotor1 = new TalonFX(61);
  private final TalonFX kShooterMotor2 = new TalonFX(62);
  private final TalonFX kBallTunnelMotor = new TalonFX(63);

  private final TalonFXConfiguration kShooter1Configuration;
  private final TalonFXConfiguration kShooter2Configuration;
  private final TalonFXConfiguration kBallTunnelConfiguration;

  private final double kShooterGoalRPS = 50.0d;
  private final double kBallTunnelVoltage = 4.0d;

  private final StatusSignal<AngularVelocity> kShooter1Velocity;
  private final StatusSignal<Voltage> kShooter1Voltage;

  private final StatusSignal<AngularVelocity> kShooter2Velocity;
  private final StatusSignal<Voltage> kShooter2Voltage;

  private final StatusSignal<Voltage> kBallTunnelVoltageSignal;

  public Robot() {
    kShooter1Configuration = new TalonFXConfiguration()
      .withSlot0(
        new Slot0Configs()
          .withKP(0.1)
          .withKV(0.3)
      )
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.Clockwise_Positive)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimit(120.0d)
          .withSupplyCurrentLimit(100.0d) 
      );
      kShooter2Configuration = new TalonFXConfiguration()
      .withSlot0(
        new Slot0Configs()
          .withKP(0.1)
          .withKV(0.3)
      )
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.CounterClockwise_Positive)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimit(120.0d)
          .withSupplyCurrentLimit(100.0d) 
      );
      kBallTunnelConfiguration = new TalonFXConfiguration()
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.Clockwise_Positive)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimit(60.0d)
          .withSupplyCurrentLimit(45.0d) 
      );

      kShooterMotor1.getConfigurator().apply(kShooter1Configuration);
      kShooterMotor2.getConfigurator().apply(kShooter2Configuration);
      kBallTunnelMotor.getConfigurator().apply(kBallTunnelConfiguration);

      kShooter1Velocity = kShooterMotor1.getVelocity();
      kShooter1Voltage = kShooterMotor1.getMotorVoltage();
          
      kShooter2Velocity = kShooterMotor2.getVelocity();
      kShooter2Voltage = kShooterMotor2.getMotorVoltage();

      kBallTunnelVoltageSignal = kBallTunnelMotor.getMotorVoltage();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Shooter 1 Velocity", kShooter1Velocity.getValueAsDouble());
    SmartDashboard.putNumber("Shooter 1 Voltage", kShooter1Voltage.getValueAsDouble());
    SmartDashboard.putNumber("Shooter 2 Velocity", kShooter2Velocity.getValueAsDouble());
    SmartDashboard.putNumber("Shooter 2 Voltage", kShooter2Voltage.getValueAsDouble());
    SmartDashboard.putNumber("Ball Tunnel Voltage", kBallTunnelVoltageSignal.getValueAsDouble());

    if(kController.getLeftBumperButton()) {
      kShooterMotor1.setControl(new VelocityVoltage(kShooterGoalRPS));
    } else {
      kShooterMotor1.setControl(new NeutralOut());
    }
    if(kController.getRightBumperButton()) {
      kShooterMotor2.setControl(new VelocityVoltage(kShooterGoalRPS));
    } else {
      kShooterMotor2.setControl(new NeutralOut());
    }
    if(kController.getXButton()) {
      kBallTunnelMotor.setControl(new VoltageOut(kBallTunnelVoltage));
    } else {
      kBallTunnelMotor.setControl(new NeutralOut());
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
