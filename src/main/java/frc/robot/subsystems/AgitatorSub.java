// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class AgitatorSub extends SubsystemBase {

  TalonFX agitatorMotor = new TalonFX(Constants.AgitatorConstants.agitatorMotorID);
  TalonFX kickerMotor = new TalonFX(Constants.AgitatorConstants.kickerMotorID);

  VoltageOut m_request = new VoltageOut(0);

  TalonFXConfiguration agitatorConfig = new TalonFXConfiguration();
  TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
  CurrentLimitsConfigs kickerlimitconfigs = new CurrentLimitsConfigs();

  /** Creates a new AgitatorSub. */
  public AgitatorSub() {
    agitatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    agitatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // placeholder
    kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kickerlimitconfigs.SupplyCurrentLimit = 30;
    kickerlimitconfigs.SupplyCurrentLimitEnable = true;

    agitatorConfig.CurrentLimits.SupplyCurrentLimit = 30;
    agitatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    agitatorMotor.getConfigurator().apply(agitatorConfig);

    kickerMotor.getConfigurator().apply(kickerConfig);
    kickerMotor.getConfigurator().apply(kickerlimitconfigs);
  }

  public void RunAgitatorAndKicker(double agitatorVoltage, double kickerVoltage) {
    agitatorMotor.setControl(m_request.withOutput(agitatorVoltage));
    kickerMotor.setControl(m_request.withOutput(kickerVoltage));
  }

  public void RunAgitator(double agitatorVoltage) {
    agitatorMotor.setControl(m_request.withOutput(agitatorVoltage));
  }

  public void RunKicker(double kickerVoltage) {
    kickerMotor.setControl(m_request.withOutput(kickerVoltage));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
