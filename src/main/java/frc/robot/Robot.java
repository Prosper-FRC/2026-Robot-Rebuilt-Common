// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final TalonFX kShooterMotor = new TalonFX(0);
  private final TalonFX kIndexerMotor = new TalonFX(1);
  private final SparkMax kHood = new SparkMax(2, MotorType.kBrushless);

  private final double kVoltage = 9.0d;
  private double kShooterRPS = 30.0d;
  private double kDesiredAngle = 0.0d;
  private final boolean kUseDesiredAngle = true;

  private final XboxController kController = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    SparkBaseConfig config = new SparkFlexConfig();
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    config.closedLoop.p(0.5);
    shooterConfig.Slot0.kP = 0.1d;
    shooterConfig.Slot0.kV = 0.365d;
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kHood.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    kShooterMotor.getConfigurator().apply(shooterConfig);
  }

  @Override
  public void robotPeriodic() {
    // Logging to smart dashboard
    SmartDashboard.putNumber("HoodPose", kHood.getEncoder().getPosition());
    SmartDashboard.putNumber("ShooterOutputVoltage", kShooterMotor.getMotorVoltage(true).getValueAsDouble());
    SmartDashboard.putNumber("ShooterVelocityRPM", kShooterMotor.getVelocity().getValueAsDouble());
    if(kUseDesiredAngle) {
      kHood.getClosedLoopController().setSetpoint(kDesiredAngle, ControlType.kPosition);
    
      if(kController.getAButton()){
        kDesiredAngle -= 0.01d;
      } else if(kController.getYButton()) {
        kDesiredAngle += 0.01d;
      }

      if(kController.getXButton()) {
        kShooterRPS += 0.05d;
      } else if (kController.getBButton()) {
        kShooterRPS -= 0.05d;
      }
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    kShooterMotor.setControl(new VelocityVoltage(kVoltage).withSlot(0));
    kIndexerMotor.setControl(new VoltageOut(-kVoltage));
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    kShooterMotor.setControl(new NeutralOut());
    kIndexerMotor.setControl(new NeutralOut());
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
