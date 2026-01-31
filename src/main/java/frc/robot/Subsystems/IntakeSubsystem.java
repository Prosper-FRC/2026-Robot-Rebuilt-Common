// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Helpers.TalonFXMotor;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
    private final TalonFXMotor intakeBar;
    private final TalonFXMotor intakeArm;

    private double intakeBarRPM;
    private double intakeArmPosition;

  public IntakeSubsystem() {
    intakeBar = new TalonFXMotor(1, new TalonFXConfiguration());
    intakeArm = new TalonFXMotor(2, new TalonFXConfiguration());

    configureTalonFX(intakeBar, false);
    configureTalonFX(intakeArm, true);

  }

  public void configureTalonFX(TalonFXMotor tfx, boolean reverse) {
    // Configure motors for inverter, idlemode, etc
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Get velocity for intake bar
    // Get position for intake arm
  }

  // Methods to control intake bar and arm
}
