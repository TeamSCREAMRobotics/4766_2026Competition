package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;
import frc.robot.constants.Constants.ShooterConstants;

public class FlywheelConfig {
  public static final TalonFXSubsystemConfiguration FLYWHEEL_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    FLYWHEEL_CONFIG.name = "Flywheel";

    FLYWHEEL_CONFIG.codeEnabled = true;
    FLYWHEEL_CONFIG.logTelemetry = false;
    FLYWHEEL_CONFIG.debugMode = false;

    FLYWHEEL_CONFIG.masterConstants =
        new TalonFXConstants(
            new CANDevice(ShooterConstants.RshooterMotorID), InvertedValue.Clockwise_Positive);

    FLYWHEEL_CONFIG.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(
              new CANDevice(ShooterConstants.LshooterMoterID),
              InvertedValue.CounterClockwise_Positive)
        };

    FLYWHEEL_CONFIG.slot0 =
        new ScreamPIDConstants(0.55, 0, 0.0)
            .getSlot0Configs(new FeedforwardConstants(0.11855, 0.31992, 0.0, 0.0));
    /* FF Characterization Results: ﻿
    Count=5540
    R2=0.99969
    kS=0.31992
    kV=0.11855 */
    //
    //
    //
    //
    FLYWHEEL_CONFIG.enableSupplyCurrentLimit = true;
    FLYWHEEL_CONFIG.supplyCurrentLimit = 40;
    FLYWHEEL_CONFIG.neutralMode = NeutralModeValue.Coast;
  }
}
