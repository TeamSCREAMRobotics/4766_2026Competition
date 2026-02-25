// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.math.Conversions;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;

public class Climber extends SubsystemBase {
  TalonFX climbermotor = new TalonFX(Constants.ClimberConstants.climbermotorID);
  VoltageOut m_request = new VoltageOut(0);
  TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
  Slot0Configs climberPIDConfigs = new Slot0Configs();
  MotionMagicConfigs climberMagicConfigs = new MotionMagicConfigs();
  MotionMagicVoltage m_MagicVoltage = new MotionMagicVoltage(0);

  /** Creates a new Climber. */
  public Climber() {
    climberConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.intakePivotForwardSoftLimit;
    climberConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.intakePivotReversSoftLimit;
    climberConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberConfigs.Feedback.SensorToMechanismRatio = 25;
    climberPIDConfigs.kP = Constants.ClimberConstants.kP;
    climberPIDConfigs.kI = Constants.ClimberConstants.kI;
    climberPIDConfigs.kD = Constants.ClimberConstants.kD;
    climberPIDConfigs.kV = Constants.ClimberConstants.kV;
    climberPIDConfigs.kG = Constants.ClimberConstants.kG;
    climberMagicConfigs.MotionMagicAcceleration = ClimberConstants.climberMagicAccereation;
    climberMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.climberMagicCruiseVelocity;

    climberConfigs.Slot0 = climberPIDConfigs;
    climberConfigs.MotionMagic = climberMagicConfigs;
    climbermotor.getConfigurator().apply(climberConfigs);
  }

  // This resets the climber position to 0
  public void resetClimberPose() {
    climbermotor.setPosition(0);
  }

  // This returns the climbers current position of the climber
  public double getClimberPose() {
    return climbermotor.getPosition(true).getValueAsDouble();
  }

  // This goes to the requested setpoint of "setpoint"
  public void climberGoToSetpoint(double setpoint) {
    climbermotor.setControl(m_MagicVoltage.withPosition(distanceToRotations(Length.fromInches(setpoint))));
  }

  // This tells you when the climber has finished going to the position given
  public boolean climberIsFinished(double setpoint) {
    return climbermotor.getPosition().getValueAsDouble() >= setpoint - 0.1
        && climbermotor.getPosition().getValueAsDouble() <= setpoint + 0.1;
  }

  public Length rotationsToDistance(){
    return Conversions.rotationsToLinearDistance(getClimberPose(), Length.fromInches(2.256 * Math.PI));
  }

  public double distanceToRotations(Length distance){
    return Conversions.linearDistanceToRotations(distance, Length.fromInches(2.256 * Math.PI));
  }

  // Dog Log(overall logging)
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("Climber Pose: ", getClimberPose());
    SmartDashboard.putNumber("Climber Pose", rotationsToDistance().getInches());
  }
}
