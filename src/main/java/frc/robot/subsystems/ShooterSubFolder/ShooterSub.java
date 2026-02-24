// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  // TalonFX LshooterMotor = new TalonFX(Constants.ShooterConstants.LshooterMoterID);
  // TalonFX RshooterMotor = new TalonFX(Constants.ShooterConstants.RshooterMotorID);
  CANrange shooterCAN = new CANrange(Constants.ShooterConstants.shooterCANID);
  CurrentLimitsConfigs shooterLimitsConfigs = new CurrentLimitsConfigs();
  MotionMagicConfigs shooterMagicConfigs = new MotionMagicConfigs();
  Slot0Configs shooterSlot0Configs = new Slot0Configs();

  TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

  VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  /** Creates a new ShooterSub. */
  public ShooterSub() {

    // shooterSlot0Configs.kP = ShooterConstants.kP;
    // shooterSlot0Configs.kI = ShooterConstants.kI;
    // shooterSlot0Configs.kD = ShooterConstants.kD;
    // shooterSlot0Configs.kV = ShooterConstants.kV;
    // shooterSlot0Configs.kG = ShooterConstants.kG;

    // shooterMagicConfigs.MotionMagicAcceleration = ShooterConstants.kMagicAcceleration;
    // shooterMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.kMagicCruiseVelocity;

    // subject to change
    shooterLimitsConfigs.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits = shooterLimitsConfigs;
    shooterConfig.MotionMagic = shooterMagicConfigs;
    shooterConfig.Slot0 = shooterSlot0Configs;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // LshooterMotor.getConfigurator().apply(shooterConfig);
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // RshooterMotor.getConfigurator().apply(shooterConfig);
  }

  public boolean hasFuel() {
    return shooterCAN.getIsDetected().getValue();
  }

  // public void runShooter(double LVelocity, double RVelocity) {
  //   //LshooterMotor.setControl(m_request.withVelocity(LVelocity));
  //   RshooterMotor.setControl(m_request.withVelocity(RVelocity));
  // }

  // public double returnLeftVelocity() {
  //   return LshooterMotor.getVelocity().getValueAsDouble();
  // }

  // public double returnRightVelocity() {
  //   return RshooterMotor.getVelocity().getValueAsDouble();
  // }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Left Velocity", returnLeftVelocity());
    // SmartDashboard.putNumber("Right Velocity", returnRightVelocity());
    // This method will be called once per scheduler run
  }
}
