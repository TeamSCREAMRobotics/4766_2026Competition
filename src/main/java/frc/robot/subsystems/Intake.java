// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
TalonFX intakeFollower = new TalonFX(Constants.IntakeConstants.intakeFollower);
TalonFX intakeMaster = new TalonFX(Constants.IntakeConstants.intakeMasterID);
VoltageOut m_request = new VoltageOut(0);
TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  /** Creates a new Intake. */
  public Intake() {
   intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
   intakeMaster.setControl(new Follower(Constants.IntakeConstants.intakeMasterID, ));
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
