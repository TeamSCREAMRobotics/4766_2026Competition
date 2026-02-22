// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  TalonFX LshooterMotor = new TalonFX(Constants.ShooterConstants.LshooterMoterID);
  TalonFX RshooterMotor = new TalonFX(Constants.ShooterConstants.RshooterMotorID);
  CANrange shooterCAN = new CANrange(Constants.ShooterConstants.shooterCANID);

  TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

  VoltageOut m_request = new VoltageOut(0);

  /** Creates a new ShooterSub. */
  public ShooterSub() {

    // subject to change

    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    LshooterMotor.getConfigurator().apply(shooterConfig);
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    RshooterMotor.getConfigurator().apply(shooterConfig);
  }

  public boolean hasFuel() {
    return shooterCAN.getIsDetected().getValue();
  }

  public void runShooter(double LVoltage, double RVoltage) {
    LshooterMotor.setControl(m_request.withOutput(LVoltage));
    RshooterMotor.setControl(m_request.withOutput(RVoltage));
  }

  public double returnLeftVelocity() {
    return LshooterMotor.getVelocity().getValueAsDouble();
  }

  public double returnRightVelocity() {
    return RshooterMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
