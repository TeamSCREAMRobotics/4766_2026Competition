// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//  package frc.robot.subsystems;
//
//  import com.ctre.phoenix6.configs.CANcoderConfiguration;
//  import com.ctre.phoenix6.configs.MotionMagicConfigs;
//  import com.ctre.phoenix6.configs.Slot0Configs;
//  import com.ctre.phoenix6.configs.TalonFXConfiguration;
//  import com.ctre.phoenix6.controls.MotionMagicVoltage;
//  import com.ctre.phoenix6.controls.VoltageOut;
//  import com.ctre.phoenix6.hardware.CANcoder;
//  import com.ctre.phoenix6.hardware.TalonFX;
//  import com.ctre.phoenix6.signals.InvertedValue;
//  import com.ctre.phoenix6.signals.NeutralModeValue;
//  import com.ctre.phoenix6.signals.SensorDirectionValue;
//  import edu.wpi.first.wpilibj2.command.SubsystemBase;
//  import frc.robot.Constants;
//  import frc.robot.Constants.ShooterConstants;
//
//  public class ShooterHood extends SubsystemBase {
//    /** Creates a new ShooterHood. */
//    TalonFX pivotMotor = new TalonFX(Constants.ShooterConstants.pivotMotorID);
//
//    CANcoder PivotEncoder = new CANcoder(Constants.ShooterConstants.pivotCanID);
//
//    Slot0Configs hoodPivotSlot0 = new Slot0Configs();
//    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
//    MotionMagicConfigs hoodPivotMagicConfigs = new MotionMagicConfigs();
//    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
//
//    VoltageOut m_request = new VoltageOut(0);
//    MotionMagicVoltage m_magicRequest = new MotionMagicVoltage(0);
//
//    public ShooterHood() {
//      encoderConfig.MagnetSensor.MagnetOffset = 0; // placeholder
//      encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
//      encoderConfig.MagnetSensor.SensorDirection =
//          SensorDirectionValue.CounterClockwise_Positive; // placeholder
//
//      PivotEncoder.getConfigurator().apply(encoderConfig);
//
//      motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
//      motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//
//      pivotMotor.getConfigurator().apply(motorConfigs);
//
//      hoodPivotSlot0.kG = ShooterConstants.kG;
//      hoodPivotSlot0.kV = ShooterConstants.kV;
//      hoodPivotSlot0.kP = ShooterConstants.kP;
//      hoodPivotSlot0.kI = ShooterConstants.kI;
//      hoodPivotSlot0.kD = ShooterConstants.kD;
//
//     pivotMotor.getConfigurator().apply(hoodPivotSlot0);
//
//      hoodPivotMagicConfigs.MotionMagicAcceleration = ShooterConstants.kMagicAcceleration;
//      hoodPivotMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.kMagicCruiseVelocity;
//
//      pivotMotor.getConfigurator().apply(hoodPivotMagicConfigs);
//    }
//
//    public void PivotToSetpoint(double setpoint) {
//      pivotMotor.setControl(m_magicRequest.withPosition(setpoint));
//  }
//
//    public void PivotToZero() {
//      pivotMotor.setControl(m_magicRequest.withPosition(0));
//    }
//
//    public boolean IsAtZero() {
//      return pivotMotor.getPosition().getValueAsDouble() <= 0.005;
//    }
//
//    public boolean IsAtSetpoint(double setpoint) {
//      return pivotMotor.getPosition().getValueAsDouble() <= setpoint + 0.005
//          && pivotMotor.getPosition().getValueAsDouble() >= setpoint - 0.005;
//    }
//
//    @Override
//    public void periodic() {
//      // This method will be called once per scheduler run
//    }
//  }
