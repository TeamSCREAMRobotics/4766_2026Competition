// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  TalonFX climbermotor = new TalonFX(Constants.ClimberConstants.climbermotorID);
  VoltageOut m_request = new VoltageOut(0);
  TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
  Slot0Configs climberPIDConfigs = new Slot0Configs();

  /** Creates a new Climber. */
  public Climber() {
    climberConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    climberConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    climberConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberPIDConfigs.kP = Constants.ClimberConstants.kP;
    climberPIDConfigs.kI = Constants.ClimberConstants.kI;
    climberPIDConfigs.kD = Constants.ClimberConstants.kD;
    climberPIDConfigs.kV = Constants.ClimberConstants.kV;
    climberPIDConfigs.kG = Constants.ClimberConstants.kG;

    climberConfigs.Slot0 = climberPIDConfigs;
    climbermotor.getConfigurator().apply(climberConfigs);
  }

  public void resetClimberPose() {
    climbermotor.setPosition(0);
  }

  public double getClimberPose() {
    return climbermotor.getPosition().getValueAsDouble();
  }

  public void climberGoToSetpoint(double setpoint) {
    climbermotor.setControl(m_request.withOutput(setpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("Climber Pose: ", getClimberPose());
  }
}
