// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private TalonFX intakePivot = new TalonFX(Constants.IntakeConstants.intakePivotID);

  private MotionMagicVoltage m_magicrequest = new MotionMagicVoltage(0);

  private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private Slot0Configs intakePIDConfigs = new Slot0Configs();
  private MotionMagicConfigs intakeMagicConfigs = new MotionMagicConfigs();

  // Creates a new Intake.
  public Intake() {

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 30;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakePIDConfigs.kP = IntakeConstants.kP;
    intakePIDConfigs.kI = IntakeConstants.kI;
    intakePIDConfigs.kD = IntakeConstants.kD;
    intakePIDConfigs.kV = IntakeConstants.kV;
    intakePIDConfigs.kG = IntakeConstants.kG;
    intakePIDConfigs.kS = IntakeConstants.kS;
    intakePIDConfigs.GravityType = GravityTypeValue.Arm_Cosine;

    intakeMagicConfigs.MotionMagicAcceleration = IntakeConstants.intakeMagicAcceleration;
    intakeMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.intakeMagicVelocity;

    intakeConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        IntakeConstants.intakePivotForwardSoftLimit;
    intakeConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        IntakeConstants.intakePivotReverseSoftLimit;
    intakeConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    intakeConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    intakePivot.getConfigurator().apply(intakeConfig);
    intakePivot.getConfigurator().apply(intakePIDConfigs);
    intakePivot.getConfigurator().apply(intakeMagicConfigs);
  }

  // Runs intake from voltage

  public void IntakeGoToSetpoint(double setpoint) {
    intakePivot.setControl(m_magicrequest.withPosition(setpoint));
  }

  public double getPivotPose() {
    return intakePivot.getPosition().getValueAsDouble();
  }

  // Reset position of intake to 0
  public void resetIntake() {
    intakePivot.setPosition(0);
  }

  public boolean isFinished(double setpoint) {
    return intakePivot.getPosition().getValueAsDouble() >= setpoint - 0.25
        && intakePivot.getPosition().getValueAsDouble() <= setpoint + 0.25;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // DogLog.log("Intake Pivot Pose", getPivotPose());
    SmartDashboard.putNumber("Intake Pivot Pose", getPivotPose());
  }
}
