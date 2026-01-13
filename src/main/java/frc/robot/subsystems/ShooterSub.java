// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  TalonFX shooterMotor = new TalonFX(Constants.ShooterConstants.shooterMoterID);
  TalonFX shooterFollower = new TalonFX(Constants.ShooterConstants.shooterFollowerID);
  //Follower might not be needed

  TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
  
  VoltageOut m_request = new VoltageOut(0);
  /** Creates a new ShooterSub. */
  public ShooterSub() {
    //subject to change
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotor.getConfigurator().apply(shooterConfig);
    shooterFollower.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public void runShooter(double voltage) {
    shooterMotor.setControl(m_request.withOutput(voltage));
  }

  public double returnVelocity() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
