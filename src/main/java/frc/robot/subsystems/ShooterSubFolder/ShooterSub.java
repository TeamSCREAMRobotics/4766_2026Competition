// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ShooterConstants;

public class ShooterSub extends SubsystemBase {
  TalonFX LshooterMotor = new TalonFX(Constants.ShooterConstants.LshooterMoterID);
  TalonFX RshooterMotor = new TalonFX(Constants.ShooterConstants.RshooterMotorID);

  CurrentLimitsConfigs shooterLimitsConfigs = new CurrentLimitsConfigs();
  MotionMagicConfigs shooterMagicConfigs = new MotionMagicConfigs();
  Slot0Configs shooterSlot0Configs = new Slot0Configs();
  TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
  TalonFXConfiguration LshooterConfig = new TalonFXConfiguration();
  TalonFXConfiguration RshooterConfig = new TalonFXConfiguration();

  VoltageOut m_request = new VoltageOut(0);

  /** Creates a new ShooterSub. */
  public ShooterSub() {

    shooterSlot0Configs.kP = ShooterConstants.kP;
    shooterSlot0Configs.kI = ShooterConstants.kI;
    shooterSlot0Configs.kD = ShooterConstants.kD;
    shooterSlot0Configs.kV = ShooterConstants.kV;
    shooterSlot0Configs.kG = ShooterConstants.kG;

    // LshooterMotor.getConfigurator().apply(shooterSlot0Configs);
    // RshooterMotor.getConfigurator().apply(shooterSlot0Configs);

    shooterMagicConfigs.MotionMagicAcceleration = ShooterConstants.kMagicAcceleration;
    shooterMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.kMagicCruiseVelocity;

    // LshooterMotor.getConfigurator().apply(shooterMagicConfigs);
    // RshooterMotor.getConfigurator().apply(shooterMagicConfigs);

    // subject to change
    shooterLimitsConfigs.StatorCurrentLimitEnable = true;

    LshooterMotor.getConfigurator().apply(shooterLimitsConfigs);
    RshooterMotor.getConfigurator().apply(shooterLimitsConfigs);

    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    LshooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    RshooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    LshooterMotor.getConfigurator().apply(shooterConfig);
    RshooterMotor.getConfigurator().apply(shooterConfig);
    LshooterMotor.getConfigurator().apply(LshooterConfig);
    RshooterMotor.getConfigurator().apply(RshooterConfig);
  }

  //  public boolean hasFuel() {
  //   return shooterCAN.getIsDetected().getValue();
  // }

  public void runShooter(double LVelocity, double RVelocity) {
    LshooterMotor.setControl(m_request.withOutput(LVelocity));
    RshooterMotor.setControl(m_request.withOutput(RVelocity));
  }

  public double returnLeftVelocity() {
    return LshooterMotor.getVelocity().getValueAsDouble();
  }

  public double returnRightVelocity() {
    return RshooterMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Velocity", returnLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", returnRightVelocity());
    // This method will be called once per scheduler run
  }
}
