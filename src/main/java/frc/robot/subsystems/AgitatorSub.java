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

public class AgitatorSub extends SubsystemBase {

  TalonFX agitatorMotor = new TalonFX(Constants.AgitatorConstants.agitatorMotorID);
  TalonFX kickerMotor = new TalonFX(Constants.AgitatorConstants.kickerMotorID);
  VoltageOut m_request = new VoltageOut(0);
  TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  /** Creates a new AgitatorSub. */
  public AgitatorSub() {
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // placeholder

    agitatorMotor.getConfigurator().apply(motorConfig);
    kickerMotor.setControl(new Follower(agitatorMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public void RunAgitator(double Voltage) {
    agitatorMotor.setControl(m_request.withOutput(Voltage));
  }

  public boolean hasFuel() {
    return true;
    // change when detection method is found
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
