// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class IntakeRoller extends SubsystemBase {
  /** Creates a new IntakeRoller. */
  private TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID);

  private TalonFX intakeFollower = new TalonFX(Constants.IntakeConstants.intakeFollowerID);

  private VoltageOut m_request = new VoltageOut(0);

  private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  public IntakeRoller() {
    intakeFollower.setControl(new Follower(intakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeMotor.getConfigurator().apply(intakeConfig);
  }

  public void runIntake(double voltage) {
    intakeMotor.setControl(m_request.withOutput(voltage));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
