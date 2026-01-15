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
import frc.robot.Constants;

// Creates the Master and the Follower and Voltage (mostly just variables)
public class Intake extends SubsystemBase {
  private TalonFX intakeFollower = new TalonFX(Constants.IntakeConstants.intakeFollower);
  private TalonFX intakeMaster = new TalonFX(Constants.IntakeConstants.intakeMasterID);
  private VoltageOut m_request =
      new VoltageOut(0); // m_request == make request (i forget what it means sometimes)
  private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  // Creates a new Intake.
  public Intake() {
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMaster.getConfigurator().apply(intakeConfig);
    intakeFollower.setControl(
        new Follower(Constants.IntakeConstants.intakeMasterID, MotorAlignmentValue.Aligned));
  }

  // Runs intake from voltage
  public void runIntake() {
    intakeMaster.setControl(m_request);
  }

  // Reset position of intake to 0
  public void resetIntake() {
    intakeMaster.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
