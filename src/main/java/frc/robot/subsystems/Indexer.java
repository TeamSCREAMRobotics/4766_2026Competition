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
import frc.robot.constants.Constants.IndexerConstants;

// TODO: Clean and make easier to read

public class Indexer extends SubsystemBase {
  TalonFX indexerMaster = new TalonFX(IndexerConstants.agitatorMotorID);
  TalonFX indexerFollower = new TalonFX(IndexerConstants.kickerMotorID);

  VoltageOut m_request = new VoltageOut(0);

  TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

  private static boolean shouldBeEnabled = true;

  /** Creates a new indexer. */
  public Indexer() {
    indexerFollower.setControl(
        new Follower(indexerMaster.getDeviceID(), MotorAlignmentValue.Opposed));

    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    indexerConfig.CurrentLimits.SupplyCurrentLimit = 20;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    indexerMaster.getConfigurator().apply(indexerConfig);
  }

  public void runIndexer(double voltage) {
    indexerMaster.setControl(m_request.withOutput(voltage));
  }

  public static void setShouldBeEnabled(boolean setEnabled) {
    shouldBeEnabled = setEnabled;
  }

  public static boolean getShouldBeEnabled() {
    return shouldBeEnabled;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
