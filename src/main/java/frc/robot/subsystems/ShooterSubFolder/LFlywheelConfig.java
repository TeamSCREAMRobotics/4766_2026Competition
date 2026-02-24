package frc.robot.subsystems.ShooterSubFolder;

import com.ctre.phoenix6.signals.InvertedValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;

public class LFlywheelConfig {
  public static final TalonFXSubsystemConfiguration LFLYWHEEL_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    LFLYWHEEL_CONFIG.name = "Flywheel";

    LFLYWHEEL_CONFIG.codeEnabled = true;
    LFLYWHEEL_CONFIG.logTelemetry = false;
    LFLYWHEEL_CONFIG.debugMode = false;

    LFLYWHEEL_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(17), InvertedValue.Clockwise_Positive);

    LFLYWHEEL_CONFIG.slot0 =
        new ScreamPIDConstants(0.1, 0.0, 0.0)
            .getSlot0Configs(new FeedforwardConstants(0.11635, 0.17347, 0.0, 0.0));

    LFLYWHEEL_CONFIG.enableSupplyCurrentLimit = true;
    LFLYWHEEL_CONFIG.supplyCurrentLimit = 20;
  }
}
